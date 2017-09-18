require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'transformer/runtime'

include Orocos

Orocos::CORBA.max_message_size = 8000000000

if not ARGV[0]
    puts "Usage: #{__FILE__} [logfiles]"
    exit
end

log = Orocos::Log::Replay.open(ARGV)
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize

# track only needed ports
velodyne_ports = log.find_all_output_ports("/base/samples/DepthMap", "laser_scans")
velodyne_ports.each do |port|
    port.tracked = true
end

# log.xsens.orientation_samples.filter do |sample|
#     sample.position.x = 0
#     sample.position.y = 0
#     sample.position.z = 0
#     sample
# end

log.transformer_broadcaster.rename('transformer_broadcaster_log') if log.has_task? 'transformer_broadcaster'

## Execute the tasks ##
Orocos.run 'depth_map_preprocessing::PointcloudConverter' => 'converter' do

    Orocos.conf.load_dir("#{ENV['AUTOPROJ_CURRENT_ROOT']}/perception/orogen/depth_map_preprocessing/scripts/config/")

    ## Configure the scan converter ##
    converter = Orocos.name_service.get 'converter'
    Orocos.conf.apply(converter, ['default'], :override => true)

    ## Connect ports with the task ##
    velodyne_ports.each do |port|
        port.connect_to converter.depth_map, :type => :buffer, :size => 100
    end

    ## Setup the transformer ##
    Orocos.transformer.load_conf("config/transforms_artemis.rb")
    Orocos.transformer.setup(converter)

    #Readline.readline

    ## Configure the tasks ##
    converter.configure

    ## Start the tasks ##
    converter.start

    Vizkit.control log

    task_inspector = Vizkit.default_loader.TaskInspector
    Vizkit.display converter, :widget => task_inspector
    Vizkit.display converter.pointcloud

    begin
        Vizkit.exec
    rescue Interrupt => e
        converter.stop
        converter.cleanup
    end
end