# helper functions
experience_mapping = function(experience_response) {
    # lowercase
    experience_response = tolower(experience_response)

    # TODO: add more experience levels
    # TODO: update strings to match qualtrics

    rating = NA
    if (experience_response == "i have never used it before") {
        rating = 1
    } else if (experience_response == "i rarely use this technology") {
        rating = 2
    } else if (experience_response == "i use it a few times a year") {
        rating = 3
    } else if (experience_response == "i use it a few times a month") {
        rating = 4
    } else if (experience_response == "i use it everyday/almost everyday") {
        rating = 5
    } else {
        print("ERROR: invalid experience response")
        print(experience_response)
        return(NA)
    }
    return(rating)
}

data_subset = function(data, modality_str) {
    # get lowercase strings
    data$Task.Name <- tolower(data$Task.Name)
    task_durations = data$Duration[data$Duration != "Duration"]
    task_names = data$Task.Name[data$Task.Name != "task name"]
    # get subset of data
    subset = task_durations[grepl(modality_str, task_names)]
    subset = as.numeric(subset)
    return(subset)
}

data_list_subset = function(data_list, modality_str) {
    subset = lapply(data_list, function(data) {
        return(data_subset(data, modality_str))
    })
    return(subset)
}

############################################################
# config
# set working directory
path_to_project <- "/Users/mlamsey/Documents/GT/Coursework/CS7633 Human-Robot Interaction/cs7633-project"
data_path <- paste(path_to_project, "analysis/data", sep = "/")
setwd(data_path)

# import qualtrics data
qualtrics_data <- read.csv("qualtrics.csv")

# import log data
setwd(data_path)

directory_list <- c("hri001", "hri002", "hri003", "hri004", "hri005", "hri006")
log_data_list <- lapply(directory_list, function(dir) {
    file_path <- paste(data_path, dir, sep = "/")
    setwd(file_path)
    i <- substr(dir, nchar(dir), nchar(dir))
    file_name <- paste0("recorded_times", i, ".csv")
    data <- read.csv(file_name)
    return(data)
})

############################################################
# basic demographics
participant_id <- qualtrics_data$Q1[3:length(qualtrics_data$Q1)]
age <- qualtrics_data$Q2[3:length(qualtrics_data$Q2)]
gender <- qualtrics_data$Q3[3:length(qualtrics_data$Q3)]
education <- qualtrics_data$Q4[3:length(qualtrics_data$Q4)]
learning_style <- qualtrics_data$Q5[3:length(qualtrics_data$Q5)]

# technology experience
tech_Q = "Q12_"
tech_Q_items = c("1", "2", "3", "4")
tech_responses = list()

for (i in 1:length(tech_Q_items)) {
    col = qualtrics_data[[paste0(tech_Q, tech_Q_items[i])]]
    
    # remove header
    col = col[3:length(col)]

    # map responses to numeric values
    ratings = as.numeric(lapply(col, experience_mapping))
    tech_responses[[i]] = ratings
}

# compute average experience per id
tech_comfort_per_user = rowMeans(do.call(cbind, tech_responses))

############################################################
# task times
task_list <- c("t", "m", "d")
mode_list <- c("gui", "xbox", "hand")
task_names <- expand.grid(mode_list, task_list)
task_names <- paste(task_names$Var1, task_names$Var2, sep = "-")

hand_times = data_list_subset(log_data_list, "hand")
xbox_times = data_list_subset(log_data_list, "xbox")
gui_times = data_list_subset(log_data_list, "gui")

hand_time_per_user = colMeans(do.call(cbind, hand_times))
xbox_time_per_user = colMeans(do.call(cbind, xbox_times))
gui_time_per_user = colMeans(do.call(cbind, gui_times))

female_hand_times = hand_time_per_user[gender == "Female"]
female_xbox_times = xbox_time_per_user[gender == "Female"]
female_gui_times = gui_time_per_user[gender == "Female"]
all_female_times = unlist(c(female_hand_times, female_xbox_times, female_gui_times))

male_hand_times = hand_time_per_user[gender == "Male"]
male_xbox_times = xbox_time_per_user[gender == "Male"]
male_gui_times = gui_time_per_user[gender == "Male"]
all_male_times = unlist(c(male_hand_times, male_xbox_times, male_gui_times))

# print(tech_comfort_per_user)
# print(hand_time_per_user)

############################################################
# plots
# tech_vs_time = data.frame(tech_comfort_per_user, hand_time_per_user, xbox_time_per_user, gui_time_per_user)
# par(mfrow = c(3, 1))
par(mfrow = c(3, 1))
# plot(tech_vs_time$tech_comfort_per_user, tech_vs_time$hand_time_per_user, xlab = "Technology Comfort", ylab = "Task Time (s)", main = "Hand")
# plot(tech_vs_time$tech_comfort_per_user, tech_vs_time$xbox_time_per_user, xlab = "Technology Comfort", ylab = "Task Time (s)", main = "Xbox")
# plot(tech_vs_time$tech_comfort_per_user, tech_vs_time$gui_time_per_user, xlab = "Technology Comfort", ylab = "Task Time (s)", main = "GUI")
plot(tech_comfort_per_user, hand_time_per_user, xlab = "Technology Comfort", ylab = "Task Time (s)", main = "Technology Comfort vs Task Completion Time", col="red", pch=19, ylim=c(0, 600))
points(tech_comfort_per_user, xbox_time_per_user, col="blue", pch=19)
points(tech_comfort_per_user, gui_time_per_user, col="green", pch=19)
grid()
legend("topright", legend=c("Hand", "Xbox", "GUI"), col=c("red", "blue", "green"), pch=19)

# plot male and female times for each mode
bins = 50 * (0:12)
hist(all_female_times, breaks=bins, col="red", main="Female Task Completion Time")
hist(all_male_times, breaks=bins, col="blue", main="Male Task Completion Time")
