/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Inspection.hpp"
#include <limits.h>

using namespace pipeline_inspection;

Inspection::Inspection(std::string const& name, TaskCore::TaskState initial_state)
    : InspectionBase(name)//, initial_state)
{
  output_frame.reset(new base::samples::frame::Frame());
}

Inspection::Inspection(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : InspectionBase(name, engine)//, initial_state)
{
  output_frame.reset(new base::samples::frame::Frame());
}

Inspection::~Inspection()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Inspection.hpp for more detailed
// documentation about them.

bool Inspection::configureHook()
{
    if (! InspectionBase::configureHook())
        return false;
    
    DetectorCalib calib;
    calib.cameraPos = _cameraPosition.get();
    calib.laserPos = _laserPosition.get();
    calib.cameraOrientation = _cameraOrientation.get();
    calib.laserNorm = _laserNorm.get();
    calib.buffer_size = _buffer_size.get();
    
    calib.min_algo = _minimizer.get();
    calib.matcher_parameter_tolerance = _matcher_parameter_tolerance.get();
    calib.matcher_value_tolerance = _matcher_value_tolerance.get();
    calib.matcher_iterations = _matcher_iterations.get();
    calib.matcher_pipe_up = _matcher_pipe_up.get();
    
    detector.init(calib);
    this->calib = calib;
    
    debug_maxY = DBL_MIN;
    debug_maxZ = debug_maxY;
    debug_minY = DBL_MAX;
    debug_minZ = debug_minY;
    
    return true;
}
bool Inspection::startHook()
{
    if (! InspectionBase::startHook())
        return false;
    return true;
}
void Inspection::updateHook()
{
    InspectionBase::updateHook();
    bool updated = false;
    
    while(_pipeline.read(lastPipe) == RTT::NewData);
    
    while(_dead_reckoning.read(lastPosition) == RTT::NewData);
    
    base::samples::LaserScan scan;
    InspectionStatus is;
    while(_laserSamples.read(scan) == RTT::NewData){
      is = detector.inspect(scan, lastPipe, lastPosition);
      _inspectionStatus.write(is);
      updated = true;
    }
    
    std::vector<base::Vector3d> points;
      while(_laserPoints.read(points) == RTT::NewData){
      is = detector.inspect(points, lastPipe, lastPosition);
      is.time = base::Time::now();
      _inspectionStatus.write(is);
      updated = true;
    }
    
    if(updated){
      _pipePoints.write(detector.getPipePoints());
      
      if(_debug && detector.getPipePoints().size() > 0){
	std::vector<base::Vector3d> points = detector.getPipePoints();
	output_frame.reset(new base::samples::frame::Frame(1000, 500));
	double minZ, maxZ, minY, maxY;
	double spanZ, spanY;
	
	minZ = debug_minZ;
	minY = debug_minY;
	maxZ = debug_maxZ;
	maxY = debug_maxY;
	
	for(std::vector<base::Vector3d>::iterator it = points.begin(); it != points.end(); it++){
	  
	  if(it->z() > maxZ)
	    maxZ = it->z();
	  
	  if(it->z() < minZ)
	    minZ = it->z();
	  
	  if(it->y() > maxY)
	    maxY = it->y();
	  
	  if(it->y() < minY)
	    minY = it->y();
	}
	
	spanZ = maxZ - minZ;
	spanY = maxY - minY;
	
	debug_maxY = maxY;
	debug_minY = minY;
	debug_maxZ = maxZ;
	debug_minZ = minZ;
	
	base::samples::frame::Frame *f = output_frame.write_access();
        const int NUM_COLS = 1000;
        const int NUM_ROWS = 500;
        
	for(std::vector<base::Vector3d>::iterator it = points.begin(); it != points.end(); it++){
	  int row = (int) (((it->z() - minZ) / spanZ) * NUM_ROWS);
	  int col = (int) (((it->y() - minY) / spanY) * NUM_COLS);	  
	  
	  if(f != 0){
	    //std::cout << f << std::endl;
	    if(row * NUM_COLS + col < NUM_COLS * NUM_ROWS){
	      f->getImagePtr()[row * NUM_COLS + col] = 255;
	    }
	    else{
	      //std::cout << "Overflow" << std::endl;
	    }	    
	    
	  }
	}
	
	f->getImagePtr()[10 * NUM_COLS + 10] = 255;
	
	if(is.laser_height > minZ && is.laser_height < maxZ){
	
          //Line height in image
          int height = (int) (((is.laser_height - minZ) / spanZ) * (NUM_ROWS - 1) );
          int count_overflow = 0;
          
          //Draw line
          for(int i = 0; i < NUM_COLS - 1; i++){
            
            if(height * NUM_COLS + i < NUM_COLS * NUM_ROWS)
              f->getImagePtr()[height * NUM_COLS + i] = 255;
            else{
              count_overflow++;
            }
                      
          }
          
          std::cout << "Overflow count: " << count_overflow << " from " << NUM_COLS << std::endl;
          std::cout << "Laser height: " << is.laser_height << std::endl;
          std::cout << "Pipe hight: " << is.pipe_height << std::endl;
          std::cout << "Pipe width: " << is.pipe_width << std::endl;
          std::cout << "Pipe radius: " << is.pipe_radius << std::endl;
          std::cout << "Z " << minZ << " - " << maxZ << std::endl;
          
          count_overflow = 0;
          if(is.pipe_width > 0.0){
          
          //Draw ellipse
            for(double i = is.pipe_center - is.pipe_width; i < is.pipe_center + is.pipe_width; i+= is.pipe_width/200){
              int col = (int) (((i - minY) / spanY) * (NUM_COLS - 1) );
              //std::cout << "i  " << i << std::endl;
              
              if(col >= 0 && col < NUM_COLS - 1){
                
                double x = (i - is.pipe_center) ;// is.pipe_width;
                //double circle = std::sqrt( 1 - (x * x) ) * is.pipe_radius; //TODO!!!
                double circle = std::sqrt( (1 - ((x*x) / (is.pipe_width * is.pipe_width)) ) * (is.pipe_height * is.pipe_height) );
                //std::cout << "Circle " << circle << std::endl; 
                int row = (int) (((is.laser_height - circle - minZ) / spanZ) * (NUM_ROWS - 1) );
                int row2 = (int) (((is.laser_height + circle - minZ) / spanZ) * (NUM_ROWS - 1) );
                
                if(row > 0 && row < NUM_ROWS -1){          
                  f->getImagePtr()[row * NUM_COLS + col] = 255;
                  count_overflow++;
                }
                  
                if(row2 > 0 && row2 < NUM_ROWS -1){
                  f->getImagePtr()[row2 * NUM_COLS + col] = 255;
                  count_overflow++;
                  
                }
              }
              
            }
            
          }
          std::cout << "Ellipse drawn" << std::endl;
          std::cout << "Ellipse overflow " << count_overflow << std::endl;
          
          if(is.pipe_height + is.pipe_radius > maxZ || is.pipe_height - is.pipe_radius < minZ){
            
            std::cout << "Pipe radius out of range" << std::endl;
            
            //Expand the frame size!!!
            
            if(is.pipe_height + is.pipe_radius > maxZ)
              debug_maxZ = is.pipe_height + is.pipe_radius + 0.00001;
            
            if(is.pipe_height - is.pipe_radius < minZ)
              debug_minZ = is.pipe_height - is.pipe_radius - 0.00001;
          }
          
        }else{
          std::cout << "Pattern out of range" << std::endl;
        }
	
	f->time = base::Time::now();
	//std::cout << "Write out" << std::endl;
	//std::cout << "Image size: " << f->getSize().width << " " << f->getSize().height << std::endl;
	//std::cout << "Y " << minY << " - " << maxY << std::endl;
	//std::cout << "Z " << minZ << " - " << maxZ << std::endl;
	output_frame.reset(f);
	_debug_frame.write(output_frame);
      }
      
    }
      
}

