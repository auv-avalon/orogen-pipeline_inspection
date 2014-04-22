require 'orocos'


require 'vizkit'

Orocos.initialize



Orocos.run 'pipeline_inspection::LaserSimulation' => "sim" ,'pipeline_inspection::Inspection' => "inspection", :valgrind => false do# ["inspection"] do
#    Orocos::log_all_ports
    #cam = Orocos::TaskContext.get "bottom_camera"

  #configure orocos

  
  sim = Orocos::TaskContext.get "sim"
  
    sim.cameraPosition = Eigen::Vector3.new(2.0, 0.0, 0.0)
    sim.cameraOrientation = Eigen::Quaternion.new(0.7071067811865476, 0.0, 0.0, 0.7071067811865476) #??? pitch 90
    sim.laserPosition = Eigen::Vector3.new(0.0, 0.0, -0.1)
    angle = 0.3
    sim.laserNorm = Eigen::Vector3.new(Math::cos(angle), 0.0, Math::sin(angle))
    
    sim.line_length = 1.0
    sim.line_height = -1.0
    sim.pipe_height = 0.15
    sim.pipe_width = 0.15
    sim.variance = 0.01
    
    sim.vehicle_velocity = [[0.5, 0.0, 0.0]]
    
    sim.configure
    sim.start
    
    inspection = Orocos::TaskContext.get 'inspection'
    inspection.cameraPosition = Eigen::Vector3.new(2.0, 0.0, 0.0)
    inspection.cameraOrientation = Eigen::Quaternion.new(0.7071067811865476, 0.0, 0.0, 0.7071067811865476) #??? pitch 90
    inspection.laserPosition = Eigen::Vector3.new(0.0, 0.0, -0.1)
    angle = 0.3
    inspection.laserNorm = Eigen::Vector3.new(Math::cos(angle), 0.0, Math::sin(angle))
    inspection.debug = true
    inspection.minimizer = :LN_NELDERMEAD #:GN_ESCH #:LN_COBYLA #:GN_DIRECT_L #:LN_NELDERMEAD
    inspection.matcher_parameter_tolerance = -0.001
    inspection.matcher_value_tolerance = -0.001
    inspection.matcher_iterations = 1000
    inspection.matcher_pipe_up = true
    
    
    inspection.pipe_color = [[0.0, 1.0, 0.0]]
    inspection.ground_color = [[1.0, 1.0, 1.0]]
    inspection.underflooding_color = [[1.0, 0.0, 0.0]]
    inspection.overflooding_color = [[0.0, 0.0, 1.0]]
    
    inspection.pipe_radius = 0.15
    inspection.pipe_tolerance = 0.2
    
    inspection.configure
    inspection.start
    
    sim.laserPoints.connect_to inspection.laserPoints
    sim.vehiclePos.connect_to inspection.dead_reckoning
    
    
    Vizkit.display sim
    Vizkit.display inspection
    Vizkit.exec
end