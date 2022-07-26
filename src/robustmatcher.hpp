/**
 * @file robustmatcher.hpp
 * @brief Robust feature matcher between two sets of binary key-points and descriptors. Added homography constrains.
 *
 * Partly based on the work included in 'OpenCV 2 Computer Vision Application Programming Cookbook'
 * https://www.packtpub.com/application-development/opencv-2-computer-vision-application-programming-cookbook
 *
 * @author Fernando Caballero, fcaballero@us.es
 * @author Francisco J Perez-Grau, fjperez@catec.aero
 * @date June 2022
 */

#ifndef __ROBUSTMATCHER_H__
#define __ROBUSTMATCHER_H__

#include <opencv2/imgproc.hpp> 
#include <opencv2/highgui.hpp> 
#include <opencv2/features2d.hpp> 
#include <opencv2/calib3d/calib3d.hpp>

/**
 * @class RobustMatcher
 * @brief Matches two sets of key-points and descriptors
 */
class RobustMatcher
{
public:
    /** @brief Enum with the different projectives model to use
     */
    enum MODEL {HOMOGRAPHY = 1, FUNDAMENTAL}; 

    /** @brief Constructor
     */
    RobustMatcher(void):
        matcher_(cv::NORM_HAMMING)
	{	
		// Variable initialization
        ratio_ = 0.65f;
        confidence_ = 0.99;
        distance_ = 3.0;
        doRansacTest_ = false;
        doRatioTest_ = false;
        refine_ = false;
        model_ = FUNDAMENTAL;
	}

    /** @brief Set ratio test
     * @param[in] ratio Ratio threshold
     */
    void setRatioTest(float ratio = 0.65f)
    {
        ratio_ = ratio;
        doRatioTest_ = true;
    }

    /** @brief Unset ratio test
     */
    void unsetRatioTest(void)
    {
        doRatioTest_ = false;
    }

    /** @brief Set RANSAC test
     * @param[in] model Projective model used: HOMOGRAPHY or FUNDAMENTAL
     * @param[in] distance Max distance in pixel to consider inlier
     * @param[in] confidence Required confidence for RANSAC
     * @param[in] refine True if want to refine the projective model
     */
    void setRansacTest(MODEL model = FUNDAMENTAL, float distance = 3, float confidence = 0.99, bool refine = false)
    {
        distance_ = distance;
        model_ = model;
        doRansacTest_ = true;
        refine_ = refine;
        confidence_ = confidence;
    }

    /** @brief Unset ratio test
     */
    void unsetRansacTest(void)
    {
        doRansacTest_ = false;
        refine_ = false;
    }

    /** @brief Match feature points using symmetry test and RANSAC. Use set/unset functions to activate/deactivate the different tests.
     * @param[in] keypoints1 Set of key-points from first image
     * @param[in] descriptors1 Set of feature descriptors from first image
     * @param[in] keypoints2 Set of key-points from second image
     * @param[in] descriptors2 Set of feature descriptors from second image
     * @param[out] matches Resulting robust matches between the first and second images
     * @return Projective matrix fitted during RANSAC test or void matrix if test was not performed
     */
    cv::Mat match(std::vector<cv::KeyPoint>& keypoints1, cv::Mat &descriptors1,
                  std::vector<cv::KeyPoint>& keypoints2, cv::Mat &descriptors2,
                  std::vector<cv::DMatch>& matches)
	{
		cv::Mat projModel;
		
        // Matching from image 1 to image 2 based on k nearest neighbours (k=2)
        std::vector<std::vector<cv::DMatch> > matches12;
        matcher_.knnMatch(descriptors1, descriptors2, matches12, 2);
		
        // Matching from image 2 to image 1 based on k nearest neighbours (k=2)
        std::vector<std::vector<cv::DMatch> > matches21;
        matcher_.knnMatch(descriptors2, descriptors1, matches21, 2);
		
        // Remove matches for which NN ratio > threshold
        if(doRatioTest_)
		{
            ratioTest(matches12);
            ratioTest(matches21);
		}
		
        // Remove non-symmetrical matches
		std::vector<cv::DMatch> symMatches;
        symmetryTest(matches12, matches21, symMatches);
		
        // Validate matches using RANSAC
        if(doRansacTest_)
			projModel = ransacTest(symMatches, keypoints1, keypoints2, matches);
		else
			matches = symMatches;
		
		return projModel;
	}

    /** @brief Clear matches for which NN ratio is greater than threshold
     * The corresponding entries in the vector of matches are cleared
     * @param matches Vector of matches
     * @return Number of removed points
     */
	int ratioTest(std::vector<std::vector<cv::DMatch> >	&matches) 
	{
		int removed=0;
        for(std::vector<std::vector<cv::DMatch> >::iterator it = matches.begin(); it!= matches.end(); ++it)
		{
            // if 2 nearest neighbors have been identified
            if(it->size() > 1)
			{
				// check distance ratio
                if((*it)[0].distance/(*it)[1].distance > ratio_)
				{
                    it->clear(); // remove match
					removed++;
				}
			} 
			else 
            {   // does not have 2 neighbors
                it->clear(); // remove match
				removed++;
			}
		}
		return removed;
	}

    /** @brief Checks that matches are present both ways
     * @param[in] matches1 Matched points in one direction
     * @param[in] matches2 Matched points in the opposite direction
     * @param[out] symMatches Output symmetrical matches
     */
    void symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
                      const std::vector<std::vector<cv::DMatch> >& matches2,
                      std::vector<cv::DMatch>& symMatches)
	{
		// for all matches image 1 -> image 2
        for(std::vector<std::vector<cv::DMatch> >::const_iterator it1= matches1.begin(); it1!= matches1.end(); ++it1)
		{
			// ignore deleted matches
            if(it1->size() < 2)
				continue;
				
			// for all matches image 2 -> image 1
            for(std::vector<std::vector<cv::DMatch> >::const_iterator it2= matches2.begin(); it2!= matches2.end(); ++it2)
			{
				// ignore deleted matches
                if(it2->size() < 2)
					continue;
				// Match symmetry test
                if((*it1)[0].queryIdx ==(*it2)[0].trainIdx &&
                        (*it2)[0].queryIdx == (*it1)[0].trainIdx)
				{
					// add symmetrical match
                    symMatches.push_back(cv::DMatch((*it1)[0].queryIdx, (*it1)[0].trainIdx, (*it1)[0].distance));
					break; // next match in image 1 -> image 2
				}
			}
		}
	}
	
    /** @brief Identify good matches using RANSAC. 
     * @param[in] matches Input matches between images 1 and 2
     * @param[in] keypoints1 Key-points from image 1
     * @param[in] keypoints2 Key-points from image 2
     * @param[out] outMatches Surviving matches after RANSAC test
     * @return Projective matrix fitted during RANSAC test 
     */
    cv::Mat ransacTest(const std::vector<cv::DMatch>& matches,
                       const std::vector<cv::KeyPoint>& keypoints1,
                       const std::vector<cv::KeyPoint>& keypoints2,
                       std::vector<cv::DMatch>& outMatches)
	{
		cv::Mat projMat;
		
        // Convert keypoints into Point2f
		std::vector<cv::Point2f> points1, points2;		
        for(std::vector<cv::DMatch>::const_iterator it= matches.begin(); it!= matches.end(); ++it)
		{
			// Get the position of left keypoints
            float x = keypoints1[it->queryIdx].pt.x;
            float y = keypoints1[it->queryIdx].pt.y;
			points1.push_back(cv::Point2f(x,y));

			// Get the position of right keypoints
            x = keypoints2[it->trainIdx].pt.x;
            y = keypoints2[it->trainIdx].pt.y;
			points2.push_back(cv::Point2f(x,y));
		}

        // Do not perform test with less than 15 matches
        if(points1.size() < 15)
            return projMat;
		
        // Compute inliers using RANSAC over the projection matrix
        std::vector<uchar> inliers(points1.size(),0);
        if(model_ == FUNDAMENTAL)
            projMat= cv::findFundamentalMat(cv::Mat(points1), cv::Mat(points2), inliers, cv::FM_RANSAC, distance_, confidence_);
        else if(model_ == HOMOGRAPHY)
            projMat = cv::findHomography(cv::Mat(points1), cv::Mat(points2), cv::RANSAC, distance_, inliers, 2000, confidence_);


        // Extract the surviving (inliers) matches
        std::vector<uchar>::const_iterator itIn = inliers.begin();
        std::vector<cv::DMatch>::const_iterator itM = matches.begin();
        for(; itIn!= inliers.end(); ++itIn, ++itM)
        {
            if(*itIn)
            { // it is a valid match
                outMatches.push_back(*itM);
            }
        }
        
        // Refine projective model if so required

        if (refine_)
        {
            // The projective matrix will be recomputed with all accepted matches
            // Convert surviving keypoints into Point2f for final F computation
            points1.clear();
            points2.clear();
            for(std::vector<cv::DMatch>::const_iterator it = outMatches.begin(); it!= outMatches.end(); ++it)
            {
                // Get the position of left keypoints
                float x = keypoints1[it->queryIdx].pt.x;
                float y = keypoints1[it->queryIdx].pt.y;
                points1.push_back(cv::Point2f(x,y));

                // Get the position of right keypoints
                x = keypoints2[it->trainIdx].pt.x;
                y = keypoints2[it->trainIdx].pt.y;
                points2.push_back(cv::Point2f(x,y));
            }

            // Compute 8-point F from all accepted matches
            if(model_ == FUNDAMENTAL)
            {
                if(points1.size() < 9)
                    return projMat;
                projMat= cv::findFundamentalMat(cv::Mat(points1), cv::Mat(points2), inliers, cv::FM_8POINT);
            }
            else if(model_ == HOMOGRAPHY)
            {
                if(points1.size() < 5)
                    return projMat;
                projMat = cv::findHomography(cv::Mat(points1), cv::Mat(points2));
            }
        }
		
        return projMat;
	}

private:

    bool doRatioTest_;      /**< Do ratio test */
    float ratio_;           /**< Max ratio between 1st and 2nd nearest neighbor*/
    bool doRansacTest_;     /**< Do RANSAC test */
    MODEL model_;            /**< Projective model used in ransac test*/
    bool refine_;           /**< If true the F/H matrix will be refined*/
    double distance_;       /**< Min distance to epipolar in RANSAC*/
    double confidence_;     /**< Confidence level (probability)*/
    cv::BFMatcher matcher_; /**< Bruteforce matcher*/
};

#endif
