/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ColorFilter.hpp"

using namespace pipeline_inspection;

ColorFilter::ColorFilter(std::string const& name, TaskCore::TaskState initial_state)
    : ColorFilterBase(name, initial_state)
{
  output_frame.reset(new base::samples::frame::Frame());
  green_frame_pointer.reset(new base::samples::frame::Frame());
  
}

ColorFilter::ColorFilter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ColorFilterBase(name, engine, initial_state)
{
  output_frame.reset(new base::samples::frame::Frame());
  green_frame_pointer.reset(new base::samples::frame::Frame());
  
}

ColorFilter::~ColorFilter()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ColorFilter.hpp for more detailed
// documentation about them.

bool ColorFilter::configureHook()
{
    if (! ColorFilterBase::configureHook())
        return false;
    return true;
}
bool ColorFilter::startHook()
{
    if (! ColorFilterBase::startHook())
        return false;
    return true;
}
void ColorFilter::updateHook()
{
    ColorFilterBase::updateHook();
    
    while(_frame_in.read(current_frame)==RTT::NewData){
    
      
      base::samples::frame::Frame frame;
      
      if(current_frame->frame_mode == base::samples::frame::MODE_BAYER_BGGR){        
        
        base::samples::frame::Frame tmp_frame;
        
        tmp_frame.init(current_frame->getWidth(), current_frame->getHeight(), current_frame->getDataDepth(), base::samples::frame::MODE_GRAYSCALE, -1);
        tmp_frame.frame_mode = base::samples::frame::MODE_GRAYSCALE;
        frame_helper::FrameHelper::convertBayerToGreenChannel(*current_frame, tmp_frame);
        
        cv::Mat green_image = frame_helper::FrameHelper::convertToCvMat(tmp_frame);

        cv::threshold(green_image, green_image, _green_threshold.get(), 255,3);
        
        base::samples::frame::Frame *green_frame = green_frame_pointer.write_access();
        frame_helper::FrameHelper::copyMatToFrame(green_image, *green_frame);
        
        green_frame_pointer.reset(green_frame);
        _green_frame.write(green_frame_pointer);
      }
      
      //std::cout << "Green out" << std::endl;
      //If image is bayer, convert to rgb
      if(current_frame->frame_mode == base::samples::frame::MODE_BAYER_BGGR){

	frame.init(current_frame->getWidth(), current_frame->getHeight(), current_frame->getDataDepth(), base::samples::frame::MODE_RGB, -1);
	frame.frame_mode = base::samples::frame::MODE_RGB;
	//frame_helper::FrameHelper::convertBayerToRGB24(current_frame->getImageConstPtr(), frame.getImagePtr(), current_frame->getWidth(),
	//		    current_frame->getHeight(), current_frame->frame_mode);
	frame_helper::FrameHelper fh;
	fh.convertColor(*current_frame, frame);
      }else{
	frame = *current_frame;
      }
      //std::cout << "COnverted" << std::endl;
      
      if(frame.frame_mode == base::samples::frame::MODE_RGB){
	  cv::Mat image; 
	  image = frame_helper::FrameHelper::convertToCvMat(frame);
	  //std::cout << "TO cv " << std::endl;
	  getGreen(image, image);
	  
	    /* Threshold optionen
	    0: Binary
	    1: Binary Inverted
	    2: Threshold Truncated
	    3: Threshold to Zero
	    4: Threshold to Zero Inverted
	  */
	  cv::threshold(image, image, _diff_threshold.get(), 255, 3);
	  
	  base::samples::frame::Frame *f = output_frame.write_access();
	  frame_helper::FrameHelper::copyMatToFrame(image, *f);
	  
	  //std::cout << "Write out diff" << std::endl;
	  output_frame.reset(f);
	  _frame_out.write(output_frame);
	  
      }else{
	std::cout << "Unsuported frame type. ONLY RGB AND BAYER_BGGR IS SUPPORTED!" << std::endl;
      }
        
    
    }
}

//http://marcosnietoblog.wordpress.com/2011/11/23/simple-highlighting-rgb-colors-with-opencv/
void ColorFilter::getGreen(cv::Mat &srcRGB, cv::Mat &desRGB){
      cv::Mat imR(srcRGB.rows, srcRGB.cols, CV_8UC1);
      cv::Mat imG(srcRGB.rows, srcRGB.cols, CV_8UC1);
      cv::Mat imB(srcRGB.rows, srcRGB.cols, CV_8UC1);
      cv::Mat imRboost(srcRGB.rows, srcRGB.cols, CV_8UC1);

      cv::Mat out[] = {imR, imG, imB};
      int from_to[] = {0,2  , 1, 1,  2, 0 };
      cv::mixChannels(&srcRGB, 1, out, 3, from_to, 3);

      cv::bitwise_not(imR, imR);
      cv::bitwise_not(imB, imB);

      cv::multiply(imG, imR, imRboost, (double)1/255);
      cv::multiply(imRboost, imB, desRGB, (double)1/255);	
}  

