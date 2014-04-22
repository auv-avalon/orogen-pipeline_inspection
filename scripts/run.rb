require 'orocos'
Orocos::CORBA::max_message_size = 7000000

require 'vizkit'

Orocos.initialize

#Orocos::MQueue::auto = true
#Orocos::MQueue::auto_sizes = true

#load log file
#log = Orocos::Log::Replay.open(ARGV[0],Typelib::Registry.new)
log = Orocos::Log::Replay.open("/home/fabio/avalon/simulation/orogen/avalon_simulation/scripts/AvalonSimulation.0.log")
#log = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/testhalle_28_3_2014/bottom_cam_cut.log")#
#log = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/testhalle_4_4_2014/bottom_cam_cut.log")

Orocos.run 'pipeline_inspection::ColorFilter' => "filter", 'pipeline_inspection::Inspection' => "inspection",
    #'structured_light::Task' => "structured_light" , :gdb => ["inspection"], :valgrind_options => ['--undef-value-errors=no'] do
    'structured_light::Task' => "structured_light" , :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do
#    Orocos::log_all_ports
    #configure orocos
    filter = Orocos::TaskContext.get 'filter'
    filter.green_threshold = 230
    filter.diff_threshold = 125
    log.bottom_camera.frame.connect_to filter.frame_in    
    filter.configure
    filter.start
    
    structured_light = Orocos::TaskContext.get 'structured_light'

    settings = structured_light.calibration_settings
    settings.path = File.expand_path(File.join(File.dirname(__FILE__),"calibration.tiff"))
    puts settings.path
    settings.start_angle = 0.48715          # + 27.91151440° --> end - 27.91151440° 
    settings.angular_resolution = -0.000974  # 0.05582°
    structured_light.calibration_settings = settings

    light_settings = structured_light.settings
    light_settings.laser_on_first = false
    #light_settings.threshold_a = 10#4
    #light_settings.threshold_b = 0#-0.04
    #light_settings.threshold_c = 0#0.0003
    light_settings.blur_x = 1
    light_settings.blur_y = 1
    #light_settings.morph_open_x = 1
    #light_settings.morph_open_y = 1
    light_settings.wavelet_size = 10
    #light_settings.min_segment_length = 1
    #structured_light.settings = light_settings    
    
    filter.frame_out.connect_to structured_light.frame
    #filter.green_frame.connect_to structured_light.frame
    #log.bottom_camera.frame.connect_to structured_light.frame
    
    structured_light.configure
    structured_light.start    
    
    inspection = Orocos::TaskContext.get 'inspection'
    inspection.cameraPosition = Eigen::Vector3.new(2.0, 0.0, 0.0)
    inspection.cameraOrientation = Eigen::Quaternion.new(0.7071067811865476, 0.0, 0.0, 0.7071067811865476) #??? pitch 90
    inspection.laserPosition = Eigen::Vector3.new(0.0, 0.0, -0.1)
    angle = 0.3
    inspection.laserNorm = Eigen::Vector3.new(Math::cos(angle), 0.0, Math::sin(angle))
    inspection.debug = true
    inspection.minimizer = :GN_ESCH #:GN_ESCH #:LN_COBYLA #:GN_DIRECT_L #:LN_NELDERMEAD
    inspection.matcher_parameter_tolerance = -0.001
    inspection.matcher_value_tolerance = -0.001
    inspection.matcher_iterations = 10000
    inspection.matcher_pipe_up = false
    inspection.configure
    inspection.start
    
    structured_light.laser_scan.connect_to inspection.laserSamples
    
    Vizkit.control log
    #Vizkit.display structured_light
    #Vizkit.display structured_light.debug_frame
    #Vizkit.display filter
    Vizkit.display inspection
    Vizkit.display inspection.debug_frame
    Vizkit.exec
end

