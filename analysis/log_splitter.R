# this file imports the time data from logs, and uses the duration of each
# task to split up the ROS log file into episodes.

# Notes:
# ManipulationControlActions:
# 0: IDLE
# 1: UP
# 2: DOWN
# 3: LEFT
# 4: RIGHT
# 5: FORWARD
# 6: BACKWARD
# 7: GRASP
# 8: RELEASE
# 
# DriveControlActions:
# 0: IDLE
# 1: FORWARD
# 2: BACKWARD
# 3: TURN_CW
# 4: TURN_CCW
# 
# ControllerModes:
# uint8 CONTROLLER_DRIVE=0
# uint8 CONTROLLER_MANIPULATION=1

# set working directory, load data
path_to_project <- "/Users/mlamsey/Documents/GT/Coursework/CS7633 Human-Robot Interaction/cs7633-project"
data_path <- paste(path_to_project, "analysis/data", sep = "/")

# log directories
directory_list <- c("hri001", "hri002", "hri003", "hri004", "hri005", "hri006")

log_file_name = "log_cleaned.csv"
time_file_name_prefix = "recorded_times"

for (i in 1:length(directory_list)){
    # set working directory
    file_path <- paste(data_path, directory_list[i], sep = "/")
    log_data = read.csv(paste(file_path, log_file_name, sep = "/"), sep = ",")
    time_data = read.csv(paste(file_path, paste(time_file_name_prefix, i, ".csv", sep = ""), sep = "/"))

    # extract and clean rows
    log_time_s = as.numeric(log_data[, "time_s"])
    log_time_s = log_time_s - log_time_s[1]
    log_time_ns = as.numeric(log_data$time_ns)
    log_time_ns = log_time_ns / 1000000000
    log_time = log_time_s + log_time_ns

    log_action = as.numeric(log_data$action)
    log_controller_state = as.numeric(log_data$controller_state)
    
    log_hand_in_frame = as.numeric(log_data$hand_in_frame)
    log_hand_in_frame = log_hand_in_frame[!is.na(log_hand_in_frame)]

    # split data into drive and manipulation halves
    log_time_drive = log_time[log_controller_state == 0]
    log_time_manipulation = log_time[log_controller_state == 1]
    log_action_drive = log_action[log_controller_state == 0]
    log_action_manipulation = log_action[log_controller_state == 1]

    # plot range
    # plot_range = 1:200
    # plot_range = 1000:1500
    # plot_range = 1:length(log_time)

    # get last time in log_time
    log_time_max = max(log_time[!is.na(log_time)])
    print(log_time_max)

    par(mfrow=c(1,1))
    plot(log_time_drive, log_action_drive, ylim=c(0, 10), xlab="Time (s)", ylab="Action", main=paste("Log Data for /", directory_list[i], sep=""), col="blue", xlim=c(0, log_time_max))
    points(log_time_manipulation, log_action_manipulation, col="red")

    print(i)
    break
}