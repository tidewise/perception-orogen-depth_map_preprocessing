require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'transformer/runtime'
require 'readline'

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

## Execute the tasks ##
Orocos.run 'depth_map_preprocessing::OutlierFilter' => 'filter' do

    Orocos.conf.load_dir("#{ENV['AUTOPROJ_CURRENT_ROOT']}/perception/orogen/depth_map_preprocessing/scripts/config/")

    ## Configure the scan filter ##
    filter = Orocos.name_service.get 'filter'
    Orocos.conf.apply(filter, ['default'], :override => true)

    ## Connect ports with the task ##
    velodyne_ports.each do |port|
        port.connect_to filter.depth_map, :type => :buffer, :size => 100
    end

    #Readline.readline

    ## Configure the tasks ##
    filter.configure

    ## Start the tasks ##
    filter.start

    Vizkit.control log

    task_inspector = Vizkit.default_loader.TaskInspector
    Vizkit.display filter, :widget => task_inspector
    Vizkit.display filter.filtered_depth_map

    begin
        Vizkit.exec
    rescue Interrupt => e
        filter.stop
        filter.cleanup
    end
end
