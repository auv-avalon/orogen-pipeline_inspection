require 'vizkit.rb'

=begin
if(ARGV.size < 1)
  path = "/media/WINDOWS/LOGS/testhalle_17_4_2014/bottom_cam_low.log"
else
  path = ARGV
end

replay = Orocos::Log::Replay.open(path)
port = replay.find_all_ports('/base/samples/frame/Frame').find do|port|
    port.number_of_samples > 0
end
=end

replay = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/testhalle_17_4_2014/bottom_cam_low.log",
                               "/media/WINDOWS/LOGS/testhalle_17_4_2014/localization_low.log",
                                "/media/WINDOWS/LOGS/testhalle_17_4_2014/avalon_back_low.log")

Orocos.initialize
Orocos.run 'offshore_pipeline_detector_test', 'line_scanner::Task' => 'line_scanner', 'pipeline_inspection::Inspection' => "inspection",
    "image_preprocessing::HSVSegmentationAndBlur" => "blur", "uw_particle_localization::MotionModel" => "model" do 
    #viz = Vizkit.default_loader.PointcloudVisualization

    task = Orocos.get 'line_scanner'
#    task.apply_conf_file('calib.yml')
    task.apply_conf_file('avalon.yml')
#    task.apply_conf_file('avalon2.yml')
    task.debug_mode = true
    
    replay.bottom_camera.frame.connect_to task.frame
    task.configure
    task.start
    
    blur = Orocos::TaskContext.get "blur"
    blur.blur = 51
    
    blur.configure
    blur.start
    
    replay.bottom_camera.frame.connect_to blur.frame
    
    
    
    
    
    pipeline_detector = Orocos::TaskContext.get 'offshore_pipeline_detector'

    Orocos.apply_conf_file(pipeline_detector, "/home/fabio/avalon/bundles/avalon/config/orogen/offshore_pipeline_detector::Task.yml")
    pipeline_detector.prefered_heading = 4.0
    pipeline_detector.maximum_check_time = 3.0
    pipeline_detector.invert_colors = false
    pipeline_detector.find_end_of_pipe = 1.0
    pipeline_detector.find_confidence = 0.5
    pipeline_detector.lose_confidence = 0.4
    pipeline_detector.weak_signal = 1.0
    
    pipeline_detector.default_distance_to_pipe = 0.5
    #pipeline_detector.apply_conf("/home/fabio/avalon/bundles/avalon/config/orogen/offshore_pipeline_detector::Task.yml")

    blur.vDebug.connect_to pipeline_detector.frame
    pipeline_detector.configure
    pipeline_detector.start   
    
    model = Orocos::TaskContext.get 'model'
    Orocos.apply_conf_file(model, "/home/fabio/avalon/bundles/avalon/config/orogen/uw_particle_localization::MotionModel.yml")
    model.configure
    model.start
    
    replay.hbridge_reader.status_samples.connect_to model.thruster_samples
    replay.depth_orientation_fusion.pose_samples.connect_to model.orientation_samples
    
    
    inspection = Orocos::TaskContext.get 'inspection'
    inspection.cameraPosition = Eigen::Vector3.new(2.0, 0.0, 0.0)
    #inspection.cameraOrientation = Eigen::Quaternion.new(0.8191520442889918, 0.0, 0.0, 0.573576436351046) #??? pitch 45
    #inspection.cameraOrientation = Eigen::Quaternion.new(0.0, 0.0, 0.0, 1.0)
    inspection.cameraOrientation = Eigen::Quaternion.new(0.7071067811865476, 0.0 , 0.0 , 0.7071067811865476) #yaw 90
    inspection.laserPosition = Eigen::Vector3.new(0.0, 0.0, -0.1)
    angle = 0.79
    inspection.laserNorm = Eigen::Vector3.new(Math::cos(angle), 0.0, Math::sin(angle))
    inspection.invert_z = true
    inspection.debug = true
    inspection.minimizer = :GN_DIRECT_L #:GN_ESCH #:LN_COBYLA #:GN_DIRECT_L #:LN_NELDERMEAD
    
    inspection.use_second_minimizer = true
    inspection.minimizer2 = :LN_NELDERMEAD
    
    inspection.matcher_parameter_tolerance = -0.001
    inspection.matcher_value_tolerance = -0.001
    inspection.matcher_iterations = 1500
    inspection.matcher_pipe_up = true
    inspection.buffer_size = 300
    
    inspection.laser_left_boundary = 0.4
    inspection.laser_right_boundary = 0.4
    
    inspection.pipe_color = [[1.0, 1.0, 1.0]]
    inspection.ground_color = [[1.0, 1.0, 1.0]]
    inspection.underflooding_color = [[1.0, 0.0, 0.0]]
    inspection.overflooding_color = [[0.0, 1.0, 1.0]]
    inspection.movement_factor = 1.0
    inspection.z_offset = 2.0 #2.0
    
    inspection.pipe_radius_v = 0.14
    inspection.pipe_radius_h = 0.09
    inspection.pipe_tolerance_v = 0.3
    inspection.pipe_tolerance_h = 0.5
    inspection.pipe_min_radius = 0.02
    
    inspection.max_pipe_angle = 1.5
    inspection.min_pipe_confidence = 0.2
    
    inspection.no_motion = false #true
    inspection.use_velocity = true
    inspection.relative_map = true #false
    
    #Orocos.apply_conf_file(inspection, "inspection.yml")
    
    inspection.configure
    inspection.start    
    
    task.pointcloud.connect_to inspection.laserPointCloud    
    pipeline_detector.pipeline.connect_to inspection.pipeline
    #replay.motion_model.pose_samples.connect_to inspection.dead_reckoning
    #replay.uw_particle_localization.dead_reckoning_samples.connect_to inspection.dead_reckoning
    model.pose_samples.connect_to inspection.dead_reckoning
    
    
    Vizkit.display inspection
    Vizkit.display task.debug
    #Vizkit.display pipeline_detector
    Vizkit.display model
    Vizkit.control replay

=begin    
    pos = 0
    task.to_async.port("pointcloud").on_data do |points|
        pos += 0.01
        points.points.each do |pt|
          pt.y += pos
          pt.z = -pt.z + 2.0
        end
        viz.updatePointCloud(points)
    end
=end

    #Vizkit.display task.debug
    Vizkit.exec
end