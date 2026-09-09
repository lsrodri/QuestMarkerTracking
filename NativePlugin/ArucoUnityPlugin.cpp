#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

#if defined(_WIN32)
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API __attribute__((visibility("default")))
#endif

extern "C" {

    struct MarkerData {
        int id;
        float posX, posY, posZ;
        float rotX, rotY, rotZ, rotW;
    };

    EXPORT_API int DetectMarkersAndEstimatePose(
        unsigned char* imageData, int width, int height, int channels,
        int dictionaryId, float markerLength,
        float fx, float fy, float cx, float cy,
        MarkerData* outMarkers, int maxOutMarkers)
    {
        if (imageData == nullptr || outMarkers == nullptr || maxOutMarkers <= 0) {
            return 0;
        }

        int type = CV_8UC4;
        if (channels == 3) type = CV_8UC3;
        else if (channels == 1) type = CV_8UC1;

        cv::Mat frame(height, width, type, imageData);

        // Convert to grayscale for detection if needed, but detectMarkers handles RGB/RGBA too.
        cv::Mat gray;
        if (channels == 4) {
            cv::cvtColor(frame, gray, cv::COLOR_RGBA2GRAY);
        } else if (channels == 3) {
            cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
        } else {
            gray = frame;
        }

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionaryId));
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<std::vector<cv::Point2f>> rejectedCandidates;

        cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        if (markerIds.empty()) {
            return 0;
        }

        // Setup camera intrinsics
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
            fx, 0, cx,
            0, fy, cy,
            0, 0, 1);
        cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F); // No distortion for Quest

        // Marker object points (center at 0,0,0)
        float halfL = markerLength / 2.0f;
        std::vector<cv::Point3f> objectPoints = {
            cv::Point3f(-halfL, halfL, 0),
            cv::Point3f(halfL, halfL, 0),
            cv::Point3f(halfL, -halfL, 0),
            cv::Point3f(-halfL, -halfL, 0)
        };

        int count = 0;
        for (size_t i = 0; i < markerIds.size() && count < maxOutMarkers; i++) {
            cv::Mat rvec, tvec;
            
            // Solve PnP for single marker
            bool success = cv::solvePnP(objectPoints, markerCorners[i], cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
            if (!success) {
                // fallback
                success = cv::solvePnP(objectPoints, markerCorners[i], cameraMatrix, distCoeffs, rvec, tvec);
            }

            if (success) {
                MarkerData& data = outMarkers[count++];
                data.id = markerIds[i];

                // Position: Unity Left-Handed (Y-up) conversion from OpenCV Right-Handed (Y-down)
                // Invert Y-axis
                data.posX = static_cast<float>(tvec.at<double>(0));
                data.posY = static_cast<float>(-tvec.at<double>(1));
                data.posZ = static_cast<float>(tvec.at<double>(2));

                // Rotation
                cv::Mat rotMat;
                cv::Rodrigues(rvec, rotMat);

                // Negate Y and Z axes of the rotation matrix to switch coordinate system handedness
                rotMat.at<double>(0, 1) = -rotMat.at<double>(0, 1);
                rotMat.at<double>(0, 2) = -rotMat.at<double>(0, 2);
                rotMat.at<double>(1, 0) = -rotMat.at<double>(1, 0);
                rotMat.at<double>(2, 0) = -rotMat.at<double>(2, 0);

                // Convert to quaternion
                double m00 = rotMat.at<double>(0, 0);
                double m01 = rotMat.at<double>(0, 1);
                double m02 = rotMat.at<double>(0, 2);
                double m10 = rotMat.at<double>(1, 0);
                double m11 = rotMat.at<double>(1, 1);
                double m12 = rotMat.at<double>(1, 2);
                double m20 = rotMat.at<double>(2, 0);
                double m21 = rotMat.at<double>(2, 1);
                double m22 = rotMat.at<double>(2, 2);

                double tr = m00 + m11 + m22;
                double qx, qy, qz, qw;

                if (tr > 0) {
                    double s = sqrt(tr + 1.0) * 2.0;
                    qw = 0.25 * s;
                    qx = (m21 - m12) / s;
                    qy = (m02 - m20) / s;
                    qz = (m10 - m01) / s;
                } else if ((m00 > m11) && (m00 > m22)) {
                    double s = sqrt(1.0 + m00 - m11 - m22) * 2.0;
                    qw = (m21 - m12) / s;
                    qx = 0.25 * s;
                    qy = (m01 + m10) / s;
                    qz = (m02 + m20) / s;
                } else if (m11 > m22) {
                    double s = sqrt(1.0 + m11 - m00 - m22) * 2.0;
                    qw = (m02 - m20) / s;
                    qx = (m01 + m10) / s;
                    qy = 0.25 * s;
                    qz = (m12 + m21) / s;
                } else {
                    double s = sqrt(1.0 + m22 - m00 - m11) * 2.0;
                    qw = (m10 - m01) / s;
                    qx = (m02 + m20) / s;
                    qy = (m12 + m21) / s;
                    qz = 0.25 * s;
                }

                data.rotX = static_cast<float>(qx);
                data.rotY = static_cast<float>(qy);
                data.rotZ = static_cast<float>(qz);
                data.rotW = static_cast<float>(qw);
            }
        }

        return count;
    }

}
