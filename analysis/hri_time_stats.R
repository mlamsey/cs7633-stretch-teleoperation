# set working directory
path_to_project <- "/Users/mlamsey/Documents/GT/Coursework/CS7633 Human-Robot Interaction/cs7633-project"
data_path <- paste(path_to_project, "analysis/data", sep = "/")
setwd(data_path)

directory_list <- c("hri001", "hri002", "hri003", "hri004", "hri005", "hri006")
data_list <- lapply(directory_list, function(dir) {
    file_path <- paste(data_path, dir, sep = "/")
    setwd(file_path)
    i <- substr(dir, nchar(dir), nchar(dir))
    file_name <- paste0("recorded_times", i, ".csv")
    data <- read.csv(file_name)
    return(data)
})

task_list <- c("t", "m", "d")
mode_list <- c("gui", "xbox", "hand")
task_names <- expand.grid(mode_list, task_list)
task_names <- paste(task_names$Var1, task_names$Var2, sep = "-")

hand_times = lapply(data_list, function(data) {
    # get lowercase strings
    data$Task.Name <- tolower(data$Task.Name)
    task_durations = data$Duration[data$Duration != "Duration"]
    task_names = data$Task.Name[data$Task.Name != "task name"]
    # get subset of data
    subset = task_durations[grepl("hand", task_names)]
    subset = as.numeric(subset)
    return(subset)
})

xbox_times = lapply(data_list, function(data) {
    # get lowercase strings
    data$Task.Name <- tolower(data$Task.Name)
    task_durations = data$Duration[data$Duration != "Duration"]
    task_names = data$Task.Name[data$Task.Name != "task name"]
    # get subset of data
    subset <- task_durations[grepl("xbox", task_names)]
    # subset to numeric
    subset <- as.numeric(subset)
    return(subset)
})

gui_times = lapply(data_list, function(data) {
    # get lowercase strings
    data$Task.Name <- tolower(data$Task.Name)
    task_durations = data$Duration[data$Duration != "Duration"]
    task_names = data$Task.Name[data$Task.Name != "task name"]
    # get subset of data
    subset = task_durations[grepl("gui", task_names)]
    subset = as.numeric(subset)
    return(subset)
})

# flatten lists
hand_times = unlist(hand_times)
xbox_times = unlist(xbox_times)
gui_times = unlist(gui_times)

# summary stats
hand_task_summary = summary(hand_times)
xbox_task_summary = summary(xbox_times)
gui_task_summary = summary(gui_times)

# print(hand_times)
# hist(hand_times[[1]], main = "Hand Times", xlab = "Time (s)", ylab = "Frequency", col = "blue")

# plot
max_t = max(c(hand_times, xbox_times, gui_times))
max_t_bin = ceiling(max_t / 100) * 100
bins <- seq(0, max_t_bin, by = 50)
par(mfrow=c(3, 1))

hist(hand_times, col="#00008B", breaks=bins, ylim=c(0, 7), main="Hand Task Completion Time", xlab="Time (s)", ylab="Frequency")
hist(xbox_times, col="#B22222", breaks=bins, ylim=c(0, 7), main="Xbox Task Completion Time", xlab="Time (s)", ylab="Frequency")
hist(gui_times, col="#9ACD32", breaks=bins, ylim=c(0, 7), main="GUI Task Completion Time", xlab="Time (s)", ylab="Frequency")

