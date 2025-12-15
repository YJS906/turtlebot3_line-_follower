#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <limits>
#include <cmath>
#include <algorithm>

class CameraViewer : public rclcpp::Node
{
public:
    CameraViewer()
    : Node("camera_viewer")
    {
        image_sub_ = image_transport::create_subscription(
            this,
            "/camera/image_raw",
            std::bind(&CameraViewer::imageCallback, this, std::placeholders::_1),
            "raw"
        );

        processed_pub_ = image_transport::create_publisher(this, "/image_processed");
        bev_pub_ = image_transport::create_publisher(this, "/camera/image_bev");
        debug_pub_ = image_transport::create_publisher(this, "/line_follower/debug_image");
        detection_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/camera/line_detection", 10);

        bottom_ratio_ = this->declare_parameter<double>("bottom_ratio", 0.3);
        threshold_value_ = this->declare_parameter<int>("threshold_value", 115);
        publish_debug_ = this->declare_parameter<bool>("publish_debug", true);
        min_contour_area_ = this->declare_parameter<double>("min_contour_area", 1000.0);
        min_thickness_pixels_ = this->declare_parameter<double>("min_thickness_pixels", 20.0);
        jump_reject_threshold_ = this->declare_parameter<double>("jump_reject_threshold", 300.0);
        line_miss_duration_ = this->declare_parameter<double>("line_miss_duration", 1.0);
        last_detection_cx_ = 0.0;
        have_last_detection_ = false;
        suppressing_ = false;
        suppression_until_ = this->now();
        jump_guard_pending_ = false;

        RCLCPP_INFO(this->get_logger(), "camera_viewer node started (bottom_ratio=%.2f, threshold=%d)", bottom_ratio_, threshold_value_);
    }

private:
    cv::Mat computeBirdEye(const cv::Mat & input)
    {
        const int w = input.cols;
        const int h = input.rows;

        std::vector<cv::Point2f> src = {
            {static_cast<float>(0.30 * w), static_cast<float>(0.70 * h)},
            {static_cast<float>(0.70 * w), static_cast<float>(0.70 * h)},
            {static_cast<float>(1.00 * w), static_cast<float>(1.00 * h)},
            {static_cast<float>(0.00 * w), static_cast<float>(1.00 * h)}};

        const int new_w = static_cast<int>(w * 1.5);
        const int new_h = static_cast<int>(h * 1.5);

        std::vector<cv::Point2f> dst = {
            {0.0F, 0.0F},
            {static_cast<float>(new_w), 0.0F},
            {static_cast<float>(new_w), static_cast<float>(new_h)},
            {0.0F, static_cast<float>(new_h)}};

        cv::Mat perspective = cv::getPerspectiveTransform(src, dst);
        cv::Mat warped;
        cv::warpPerspective(input, warped, perspective, cv::Size(new_w, new_h));
        return warped;
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        // Convert ROS Image → OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        // Flip 180 degrees to create processed image
        cv::Mat processed_image;
        cv::rotate(cv_ptr->image, processed_image, cv::ROTATE_180);
        cv_bridge::CvImage processed_msg(msg->header, "bgr8", processed_image);
        processed_pub_.publish(processed_msg.toImageMsg());

        // Bird-eye view transform from processed image
        cv::Mat bev = computeBirdEye(processed_image);
        cv_bridge::CvImage bev_msg(msg->header, "bgr8", bev);
        bev_pub_.publish(bev_msg.toImageMsg());

        // Threshold to find dark line (invert). If threshold_value_ <= 0, use Otsu.
        cv::Mat gray, mask;
        cv::cvtColor(bev, gray, cv::COLOR_BGR2GRAY);
        const double thresh_value = threshold_value_ > 0 ? threshold_value_ : 0.0;
        const int thresh_type = threshold_value_ > 0 ? cv::THRESH_BINARY_INV : (cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        cv::threshold(gray, mask, thresh_value, 255, thresh_type);

        geometry_msgs::msg::Vector3 detection;

        const double center = static_cast<double>(mask.cols) / 2.0;
        const int start_row = static_cast<int>(mask.rows * (1.0 - bottom_ratio_));
        cv::Mat bottom_band = mask.rowRange(start_row, mask.rows);
        const double coverage = static_cast<double>(cv::countNonZero(bottom_band)) /
            static_cast<double>(bottom_band.total());
        detection.y = coverage;

        std::vector<std::vector<cv::Point>> contours;
        cv::Mat mask_for_contours = mask.clone();
        cv::findContours(mask_for_contours, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        struct CandidateInfo
        {
            double cx;
            double cy;
            double area;
            bool touches_bottom;
        };
        std::vector<CandidateInfo> candidates;
        candidates.reserve(contours.size());

        for (const auto & contour : contours) {
            double area = cv::contourArea(contour);
            if (area < min_contour_area_) {
                continue;
            }
            double perimeter = cv::arcLength(contour, true);
            // Reject skinny crack-like blobs using average thickness (2A/P).
            double thickness = perimeter > 0.0 ? (2.0 * area / perimeter) : 0.0;
            if (thickness < min_thickness_pixels_) {
                continue;
            }
            cv::Moments cm = cv::moments(contour);
            if (cm.m00 <= 0.0) {
                continue;
            }
            double cx = cm.m10 / cm.m00;
            double cy = cm.m01 / cm.m00;

            const bool touches_bottom = std::any_of(
                contour.begin(),
                contour.end(),
                [start_row](const cv::Point & point) {
                    return point.y >= start_row;
                });

            candidates.push_back({cx, cy, area, touches_bottom});
        }

        const bool multiple_candidates = candidates.size() > 1;
        const CandidateInfo * chosen_candidate = nullptr;
        if (!candidates.empty()) {
            // Prefer contours that reach the bottom band, choosing the rightmost if multiple exist.
            const CandidateInfo * rightmost_bottom = nullptr;
            for (const auto & candidate : candidates) {
                if (candidate.touches_bottom &&
                    (!rightmost_bottom || candidate.cx > rightmost_bottom->cx)) {
                    rightmost_bottom = &candidate;
                }
            }

            if (rightmost_bottom) {
                chosen_candidate = rightmost_bottom;
            } else if (have_last_detection_) {
                double best_score = std::numeric_limits<double>::max();
                for (const auto & candidate : candidates) {
                    double score = std::abs(candidate.cx - last_detection_cx_);
                    if (score < best_score) {
                        best_score = score;
                        chosen_candidate = &candidate;
                    }
                }
            } else {
                const CandidateInfo * largest_area = nullptr;
                for (const auto & candidate : candidates) {
                    if (!largest_area || candidate.area > largest_area->area) {
                        largest_area = &candidate;
                    }
                }
                chosen_candidate = largest_area;
            }
        }

        const rclcpp::Time now = this->now();
        bool jump_guard_active = jump_guard_pending_;
        jump_guard_pending_ = false;
        bool currently_suppressed = suppressing_ && now < suppression_until_;
        if (!currently_suppressed && suppressing_) {
            suppressing_ = false;
        }

        if (jump_guard_active && chosen_candidate && have_last_detection_) {
            double jump = std::abs(chosen_candidate->cx - last_detection_cx_);
            if (jump > jump_reject_threshold_) {
                suppressing_ = true;
                suppression_until_ = now + rclcpp::Duration::from_seconds(line_miss_duration_);
                currently_suppressed = true;
                chosen_candidate = nullptr;
                have_last_detection_ = false;
            }
        }

        if (currently_suppressed) {
            detection.x = 0.0;
            detection.z = 0.0;
        } else if (chosen_candidate) {
            detection.x = chosen_candidate->cx - center;
            detection.z = 1.0;  // valid
            last_detection_cx_ = chosen_candidate->cx;
            have_last_detection_ = true;
            if (multiple_candidates && !currently_suppressed) {
                jump_guard_pending_ = true;
            }
        } else {
            detection.x = 0.0;
            detection.z = 0.0;  // invalid
            have_last_detection_ = false;
        }

        detection_pub_->publish(detection);

        if (publish_debug_) {
            cv::Mat overlay;
            cv::cvtColor(mask, overlay, cv::COLOR_GRAY2BGR);
            cv::addWeighted(bev, 0.6, overlay, 0.4, 0.0, overlay);
            if (chosen_candidate && !currently_suppressed) {
                cv::circle(overlay, cv::Point(static_cast<int>(std::round(chosen_candidate->cx)),
                                              static_cast<int>(std::round(chosen_candidate->cy))),
                           8, cv::Scalar(0, 0, 255), 2);
                cv::line(overlay,
                         cv::Point(static_cast<int>(center), overlay.rows - 1),
                         cv::Point(static_cast<int>(chosen_candidate->cx), overlay.rows - 1),
                         cv::Scalar(0, 255, 0), 2);
            }
            cv_bridge::CvImage debug_msg(msg->header, "bgr8", overlay);
            debug_pub_.publish(debug_msg.toImageMsg());
        }
    }

    image_transport::Subscriber image_sub_;
    image_transport::Publisher processed_pub_;
    image_transport::Publisher bev_pub_;
    image_transport::Publisher debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr detection_pub_;
    double bottom_ratio_;
    int threshold_value_;
    bool publish_debug_;
    double min_contour_area_;
    double min_thickness_pixels_;
    double jump_reject_threshold_;
    double line_miss_duration_;
    double last_detection_cx_;
    bool have_last_detection_;
    bool suppressing_;
    rclcpp::Time suppression_until_;
    bool jump_guard_pending_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraViewer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
