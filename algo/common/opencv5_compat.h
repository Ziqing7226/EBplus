// algo/common/opencv5_compat.h
//
// OpenCV 5.0 split calib3d into calib/geometry/stereo/ptcloud and moved the
// chessboard/circle-grid detection (findChessboardCorners, drawChessboard-
// Corners, findCirclesGrid, CALIB_CB_*, SimpleBlobDetector) into objdetect,
// while the 2D geometry helpers (convexHull, contourArea, boundingRect,
// minAreaRect, ...) left imgproc for geometry/2d.hpp. The calib3d.hpp shim
// shipped by some 5.x packagings does not pull the new headers in, so call
// sites must include the version-correct set explicitly — this header does
// it once, centrally. Include it instead of calib3d.hpp when any of those
// APIs is used; <opencv2/core.hpp> must come first so CV_MAJOR_VERSION is
// defined (this header includes it itself).
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#if CV_MAJOR_VERSION >= 5
#include <opencv2/objdetect.hpp>
#include <opencv2/calib.hpp>
#include <opencv2/geometry.hpp>
#include <opencv2/geometry/2d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
