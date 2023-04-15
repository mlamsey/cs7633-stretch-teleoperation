get_XGH_times = function(data) {
    # get lowercase strings
    data$Task.Name <- tolower(data$Task.Name)
    task_durations = as.numeric(data$Duration[data$Duration != "Duration"])
    task_names = data$Task.Name[data$Task.Name != "task name"]
    # get subset of data
    hand_subset = task_durations[grepl("hand", task_names)]
    h_time = sum(hand_subset)
    xbox_subset = task_durations[grepl("xbox", task_names)]
    x_time = sum(xbox_subset)
    gui_subset = task_durations[grepl("gui", task_names)]
    g_time = sum(gui_subset)

    XGH_times = c(x_time, g_time, h_time)
    
    return(XGH_times)
}

# set working directory, load data
path_to_project <- "/Users/mlamsey/Documents/GT/Coursework/CS7633 Human-Robot Interaction/cs7633-project"
data_path <- paste(path_to_project, "analysis/data", sep = "/")

# log directories
directory_list <- c("hri001", "hri002", "hri003", "hri004", "hri005", "hri006")
time_file_name = "recorded_times_cleaned.csv"

time_data_list = list()

for (i in 1:length(directory_list)) {
    d = directory_list[i]
    setwd(paste(data_path, d, sep = "/"))
    time_data_list[[i]] = read.csv(time_file_name)
}

XGH_task_times = lapply(time_data_list, get_XGH_times)

xbox_task_times = lapply(XGH_task_times, function(x) x[1])
gui_task_times = lapply(XGH_task_times, function(x) x[2])
hand_task_times = lapply(XGH_task_times, function(x) x[3])

# xbox task times
xbox_first = c(5, 6)
xbox_second = c(2, 4)
xbox_third = c(1, 3)
x_first_mean = mean(as.numeric(xbox_task_times[xbox_first]))
x_second_mean = mean(as.numeric(xbox_task_times[xbox_second]))
x_third_mean = mean(as.numeric(xbox_task_times[xbox_third]))
xbox_ordering_average_times = c(x_first_mean, x_second_mean, x_third_mean)

# gui task times
gui_first = c(3, 4)
gui_second = c(1, 5)
gui_third = c(2, 6)
g_first_mean = mean(as.numeric(gui_task_times[gui_first]))
g_second_mean = mean(as.numeric(gui_task_times[gui_second]))
g_third_mean = mean(as.numeric(gui_task_times[gui_third]))
gui_ordering_average_times = c(g_first_mean, g_second_mean, g_third_mean)

# hand task times
hand_first = c(1, 2)
hand_second = c(3, 6)
hand_third = c(4, 5)
h_first_mean = mean(as.numeric(hand_task_times[hand_first]))
h_second_mean = mean(as.numeric(hand_task_times[hand_second]))
h_third_mean = mean(as.numeric(hand_task_times[hand_third]))
hand_ordering_average_times = c(h_first_mean, h_second_mean, h_third_mean)

par(mfrow=c(1,1))
plot(hand_ordering_average_times, type = "b", col="#00008B", xlab = "Ordering", ylab = "Time (s)", main = "Time to Complete All Tasks", xaxt='n', ylim=c(0,1400))
points(xbox_ordering_average_times, type = "b", col = "#B22222")
points(gui_ordering_average_times, type = "b", col = "#9ACD32")
axis(1, at = 1:3, labels = c("First Task", "Second Task", "Third Task"))
grid()
legend("topright", legend = c("Xbox", "GUI", "Hand"), col = c("#B22222", "#9ACD32", "#00008B"), lty = 1, cex = 0.8)
