/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "LaserSimulation.hpp"

using namespace pipeline_inspection;

LaserSimulation::LaserSimulation(std::string const& name, TaskCore::TaskState initial_state)
    : LaserSimulationBase(name, initial_state)
{
}

LaserSimulation::LaserSimulation(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : LaserSimulationBase(name, engine, initial_state)
{
}

LaserSimulation::~LaserSimulation()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See LaserSimulation.hpp for more detailed
// documentation about them.

bool LaserSimulation::configureHook()
{
    if (! LaserSimulationBase::configureHook())
        return false;
    return true;
}
bool LaserSimulation::startHook()
{
    if (! LaserSimulationBase::startHook())
        return false;
    
    noise = new boost::variate_generator<boost::mt19937, boost::normal_distribution<> >(boost::mt19937(time(0)),
           boost::normal_distribution<>(0.0,  _variance.get()));    
    
    
    return true;
}
void LaserSimulation::updateHook()
{
    LaserSimulationBase::updateHook();
    
    createPoints();
    projectToLaserPlane();
    convertToCameraFrame();
    createNoise();
    
    _laserPoints.write(points);
    
    base::samples::Pointcloud pc;
    pc.time = base::Time::now();
    pc.points = points;
    _laserPointCloud.write(pc);
    
}

void LaserSimulation::createPoints(){
  points.clear();
  
  double line_height = _line_height.get();
  double line_begin = -0.5 * _line_length.get();
  double line_end = 0.5 * _line_length.get();
  double line_length = _line_length.get();
  double pipe_center = 0.0;
  double pipe_begin = pipe_center - (_pipe_width.get() * 0.5);
  double pipe_end = pipe_center + (_pipe_width.get() * 0.5);
  double pipe_height = _pipe_height.get();
  double pipe_width = _pipe_width.get();
  
  for(double i = line_begin; i < line_end; i += (line_length / 100) ){
    
    if(i < pipe_begin || i > pipe_end){
      points.push_back(base::Vector3d(_cameraPosition.get().x(), i, line_height) );
    
    }else{
      
      double x = std::fabs( pipe_center - i);
      double c = std::sqrt( (1 - ( (x * x) / (pipe_width * pipe_width) ) ) * (pipe_height * pipe_height) );
      
      points.push_back(base::Vector3d(_cameraPosition.get().x(), i , line_height + c ) );
    }
    
  }  
  
}

void LaserSimulation::projectToLaserPlane(){
  
  base::Vector3d laserNorm = _laserNorm.get();
  base::Vector3d laserPos = _laserPosition.get();
  
  for(std::vector<base::Vector3d>::iterator it = points.begin(); it != points.end(); it++){
    
    base::Vector3d pointL;
    
    pointL = *it;
    pointL -= ( ( (pointL - laserPos).dot(laserNorm) ) / laserNorm.dot( laserNorm) ) * laserNorm;
    
    *it = pointL;
  } 
  
}


void LaserSimulation::convertToCameraFrame(){
  
  base::Quaterniond rot = _cameraOrientation.get().inverse();
  
  for(std::vector<base::Vector3d>::iterator it = points.begin(); it != points.end(); it++){
    //base::Vector3d pointC = (rot * *it) - _cameraPosition.get();
    base::Vector3d pointC = rot * (*it - _cameraPosition.get() );
    *it = pointC;
  }
  
}


void LaserSimulation::createNoise(){
  
  for(std::vector<base::Vector3d>::iterator it = points.begin(); it != points.end(); it++){
    
    it->x() += (*noise)();
    it->y() += (*noise)();
    it->z() += (*noise)();
    
  } 
}

