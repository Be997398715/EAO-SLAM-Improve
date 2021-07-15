#ifndef _YOLOV3_H_
#define _YOLOV3_H_

//#define DEEP_SORT 1
//#define SORT 1

#ifdef DEEP_SORT
    //////deep-sort
    #include <unistd.h>
    #include "deepsort.h"
    #include "VCATime.h"
    #include "VCAImage.h"
    #include <gflags/gflags.h>
#endif

#ifdef SORT
    //////sort
    #include "tracker.h"
    #include "utils.h"
#endif


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>
#include<time.h>
using namespace std;
using namespace cv;
//using namespace cv::dnn;  //comflict with modeler.h


namespace ORB_SLAM2
{
class BoxSE : public cv::Rect
{
public:
	int m_class = -1;			// class id.
	float m_score = 0.0F;		// probability.
	std::string m_class_name;	// class name.

    int m_track_id = -1;        //track id

	BoxSE()
	{
		m_class_name = "Unknown";
	}

	BoxSE(int c, float s, int track_id, int _x, int _y, int _w, int _h, std::string name = "")
		:m_class(c), m_score(s), m_track_id(track_id)
	{
		this->x = _x;
		this->y = _y;
		this->width = _w;
		this->height = _h;
		char const *lb[5] = { "th","st","nd","rd","th" };

		if (name.length() == 0)
		{
			m_class_name = std::to_string(m_class) + lb[m_class < 4 ? m_class : 4] + " class";
		}
	}
};


class YOLO
{
private:
    const string image_path = "/home/user/project/EAO-SLAM-master-improve/test.png";//待检测图片
    const string darknet_cfg = "/home/user/project/cuda-orbslam2-opencv4-yolov3-sorttrack_deepsort/yolov3.cfg";//网络文件
    const string darknet_weights = "/home/user/project/cuda-orbslam2-opencv4-yolov3-sorttrack_deepsort/yolov3.weights";//训练模型
    const string class_names = "/home/user/project/cuda-orbslam2-opencv4-yolov3-sorttrack_deepsort/coco.names";
    std::vector<std::string> class_labels ;//类标签

    cv::dnn::Net net_yolo;

    // Initialize the parameters
    float confThreshold = 0.5; // Confidence threshold
    float nmsThreshold = 0.4;  // Non-maximum suppression threshold
    int inpWidth = 608;        // Width of network's input image
    int inpHeight = 608;       // Height of network's input image

    vector<int> classIds,classIds_nms;
    vector<float> confidences;
    vector<Rect> boxes,boxes_nms;

    #ifdef DEEP_SORT
        DS_Tracker h_tracker;
        DS_DetectObject temp_object;
        DS_DetectObjects detect_objects;
        DS_TrackObjects track_objects;
    #endif

    #ifdef SORT
        Tracker_ tracker;
        vector<Rect> boxes_nms_temp;
    #endif

    vector<String> names;


    // Get the names of the output layers
    vector<String> getOutputsNames(const cv::dnn::Net& net)
    {
	    static vector<String> names;
	    if (names.empty())
	    {
		    //Get the indices of the output layers, i.e. the layers with unconnected outputs
		    vector<int> outLayers = net.getUnconnectedOutLayers();

		    //get the names of all the layers in the network
		    vector<String> layersNames = net.getLayerNames();

		    // Get the names of the output layers in names
		    names.resize(outLayers.size());
		    for (size_t i = 0; i < outLayers.size(); ++i){
			    names[i] = layersNames[outLayers[i] - 1];
			    cout << names[i] <<endl;
			}
	    }
	    cout << "names yolo" <<endl;

	    return names;
    }


    // Draw the predicted bounding box
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
    {
	    //Draw a rectangle displaying the bounding box
	    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 0, 255));

	    //Get the label for the class name and its confidence
	    string label = format("%.2f", conf);
	    if (!class_labels.empty())
	    {
		    //assert(classId < (int)classes.size());
		    label = class_labels[classId] + ":" + label;
	    }

    	//Display the label at the top of the bounding box
	    int baseLine;
	    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	    top = max(top, labelSize.height);
	    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
    }


    // Remove the bounding boxes with low confidence using non-maxima suppression
    void postprocess(Mat& frame, const vector<Mat>& outs, vector<vector<int>>& _mat)
    {

	    for (size_t i = 0; i < outs.size(); ++i)
	    {
		    // Scan through all the bounding boxes output from the network and keep only the
		    // ones with high confidence scores. Assign the box's class label as the class
		    // with the highest score for the box.
		    float* data = (float*)outs[i].data;

		    for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
		    {
			    Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
			    Point classIdPoint;
			    double confidence;
			    // Get the value and location of the maximum score
			    minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
			    if (confidence > confThreshold)
			    {
				    int centerX = (int)(data[0] * frame.cols);
				    int centerY = (int)(data[1] * frame.rows);
				    int width = (int)(data[2] * frame.cols);
				    int height = (int)(data[3] * frame.rows);
				    int left = centerX - width / 2;
				    int top = centerY - height / 2;

	    			classIds_nms.push_back(classIdPoint.x);
		    		confidences.push_back((float)confidence);
			    	boxes_nms.push_back(Rect(left, top, width, height));

			    }
		    }
	    }

	    // Perform non maximum suppression to eliminate redundant overlapping boxes with
	    // lower confidences
	    vector<int> indices;
	    cv::dnn::NMSBoxes(boxes_nms, confidences, confThreshold, nmsThreshold, indices);
	    for (size_t i = 0; i < indices.size(); ++i)
	    {
		    int idx = indices[i];
		    Rect box = boxes_nms[idx];
		    #ifdef SORT
		        boxes_nms_temp.push_back(boxes_nms[idx]);
		    #endif
		    //classIds.push_back(classIds_nms[idx]);
		    drawPred(classIds_nms[idx], confidences[idx], box.x, box.y,
			box.x + box.width, box.y + box.height, frame);
		    #ifdef DEEP_SORT
		    	temp_object.class_id=classIds_nms[idx];
		    	temp_object.confidence=confidences[idx];
		    	temp_object.rect.x=box.x;
		    	temp_object.rect.y=box.y;
		    	temp_object.rect.width=box.width;
		    	temp_object.rect.height=box.height;
		    	detect_objects.push_back(temp_object);
		    #endif
		    vector<int> result;

            result.push_back(classIds_nms[idx]);
            result.push_back(box.x);
            result.push_back(box.y);
            result.push_back(box.width);
            result.push_back(box.height);
            result.push_back(int(confidences[idx]));
            _mat.push_back(result);
	    }
    }

public:
    YOLO(){
        // 加载模型
	    this->net_yolo = cv::dnn::readNetFromDarknet(darknet_cfg, darknet_weights);

	    //net.setPreferableBackend(DNN_BACKEND_INFERENCE_ENGINE);
	    //net.setPreferableTarget(DNN_TARGET_GPU);

	    //this->net.setPreferableBackend(DNN_BACKEND_OPENCV);
	    //this->net.setPreferableTarget(DNN_TARGET_CPU);

        this->net_yolo.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        this->net_yolo.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);

	    // 加载标签集
	    //std::vector<std::string> classLabels;
	    ifstream classNamesFile(class_names);
	    if (classNamesFile.is_open())
	    {
		    string className = "";
		    while (std::getline(classNamesFile, className))
			    this->class_labels.push_back(className);
	    }

        this->names = this->getOutputsNames(this->net_yolo);

        // create DEEP-SORT tracker
        #ifdef DEEP_SORT
            h_tracker = DS_Create();
            if(NULL==h_tracker)
            {
                printf("DS_CreateTracker error.\n");

           }
        #endif

	    // create SORT tracker
	    #ifdef SORT

        #endif
    }

    vector<vector<int>> detect(const Mat &imRGB){
        /*self*/
        #ifdef DEEP_SORT
            detect_objects.clear();
        #endif
        #ifdef SORT
            boxes_nms_temp.clear();
        #endif

        this->classIds.clear();
        this->classIds_nms.clear();
        this->confidences.clear();
        this->boxes.clear();
        this->boxes_nms.clear();

        cv::Mat img = imRGB.clone();

        cv::Mat blob = cv::dnn::blobFromImage(img, 1.0 / 255.0, { 608,608 }, 0.00392, true);
        this->net_yolo.setInput(blob);

        // 检测
        vector<Mat> detectionMat;

        this->net_yolo.forward(detectionMat, this->names);// 6 845 1 W x H x C
        // Remove the bounding boxes with low confidence
        vector<vector<int>> _mat;
        this->postprocess(img, detectionMat, _mat);

        /*** Run DEEP-SORT tracker Support.***/
        #ifdef DEEP_SORT
            DS_Update(h_tracker, detect_objects, track_objects);
            cv::Mat img_tracking = imRGB.clone();

            _mat.clear();   //clear output bacause of using track oebjects attributes next.

            for(auto oloop : track_objects)
            {
    //			oloop.track_id, oloop.class_id, oloop.confidence,
    //				oloop.rect.x,oloop.rect.y,oloop.rect.width,oloop.rect.height);
    //
                vector<int> result;

                result.push_back(oloop.class_id);
                result.push_back(oloop.rect.x);
                result.push_back(oloop.rect.y);
                result.push_back(oloop.rect.width);
                result.push_back(oloop.rect.height);
                result.push_back(int(oloop.confidence));
                result.push_back(oloop.track_id);
                _mat.push_back(result);

                string text = std::to_string(oloop.class_id) + "-" + std::to_string(oloop.track_id);
                Rect bbox(oloop.rect.x,oloop.rect.y,oloop.rect.width,oloop.rect.height);
                cv::putText(img_tracking, text, cv::Point(oloop.rect.x, oloop.rect.y - 10),
                            cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 255, 255), 1);
                cv::rectangle(img_tracking, bbox, cv::Scalar(0, 255, 255), 1);
                boxes.push_back(bbox);              //track position xywh
                classIds.push_back(oloop.track_id); //track id
            }
            cv::imshow("Tracking", img_tracking);
        #endif

        /*** Run SORT tracker Not Support because no confidence and poor track ability.***/
        #ifdef SORT
            auto frame_index = ni + 1;
            cv::Mat img_tracking = imRGB.clone();
            const auto &detections = boxes_nms_temp;
            tracker.Run(detections);
            const auto tracks = tracker.GetTracks();
            /*** Tracker update done ***/
            for (auto &trk : tracks) {
                // only draw tracks which meet certain criteria
                if (trk.second.coast_cycles_ < kMaxCoastCycles &&
                    (trk.second.hit_streak_ >= kMinHits || frame_index < kMinHits)) {
                    const auto &bbox = trk.second.GetStateAsBbox();
                    cv::putText(img_tracking, std::to_string(trk.first), cv::Point(bbox.tl().x, bbox.tl().y - 10),
                                cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 255, 255), 1);
                    cv::rectangle(img_tracking, bbox, cv::Scalar(0, 255, 255), 1);
                    //cout << "bbox : " << bbox<< endl;
                    boxes.push_back(bbox);           //track position xywh
                    classIds.push_back(trk.first);   //track id
                }
            }
            cv::imshow("Tracking", img_tracking);
        #endif


        // Put efficiency information. The function getPerfProfile returns the
        // overall time for inference(t) and the timings for each of the layers(in layersTimes)
        vector<double> layersTimes;
        double freq = getTickFrequency() / 1000;
        double t = this->net_yolo.getPerfProfile(layersTimes) / freq;
        string label = format("Inference time for a frame : %.2f ms", t);
        putText(img, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

        // Write the frame with the detection boxes
        Mat detectedFrame;
        img.convertTo(detectedFrame, CV_8U);
        // 显示图片
        cv::imshow("YoloV3-detect", detectedFrame);

        return _mat;
    }

};

}// namespace ORB_SLAM

#endif