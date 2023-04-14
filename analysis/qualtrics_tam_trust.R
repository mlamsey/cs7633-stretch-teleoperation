# helper functions
likert_to_number = function(likert_response, bool_positive=TRUE) {
    # lowercase
    likert_response = tolower(likert_response)

    rating = NA
    if (likert_response == "strongly disagree") {
        rating = 1
    } else if (likert_response == "disagree") {
        rating = 2
    } else if (likert_response == "neutral") {
        rating = 3
    } else if (likert_response == "agree") {
        rating = 4
    } else if (likert_response == "strongly agree") {
        rating = 5
    } else {
        print("ERROR: invalid likert response")
        print(likert_response)
        return(NA)
    }
    if (bool_positive) {
        return(rating)
    } else {
        return(6 - rating)
    }
}

# config
n_responses = 6

# set working directory
path_to_project <- "/Users/mlamsey/Documents/GT/Coursework/CS7633 Human-Robot Interaction/cs7633-project"
data_path <- paste(path_to_project, "analysis/data", sep = "/")
setwd(data_path)

# import qualtrics data
qualtrics_data <- read.csv("qualtrics.csv")

# extract Likert questions
# questions = qualtrics_data[, 18:ncol(qualtrics_data)]
# question_labels = lapply(questions, function(question) {
#     question_label = question[1]
#     return(question_label)
# })

# notes
# hand questions: 6, 7
# xbox questions: 8, 9
# gui questions: 10, 11
# 1a: i would like to use this system frequently
# 2a: i found the system unnecessarily complex
# 3a: i thought the system was easy to use
# 4a: i think that i would need the support of a technical person to be able to use this system
# 5a: i found the various functions in this system were well integrated
# 6a: i thought there was too much inconsistency in this system
# 7a: i would imagine that most people would learn to use this system very quickly
# 8a: i found the system very cumbersome to use
# 9a: i felt very confident using the system
# 10a: i needed to learn a lot of things before i could get going with this system
# 1b: i trust the system
# 2b: this system is reliable
# 3b: i thought the system was easy to use

#                       1a    2a     3a    4a     5a    6a     7a    8a     9a    10a    1b    2b    3b
question_positivity = c(TRUE, FALSE, TRUE, FALSE, TRUE, FALSE, TRUE, FALSE, TRUE, FALSE, TRUE, TRUE, TRUE)

# get question labels
hand_prefix_ids = c(6, 7)
xbox_prefix_ids = c(8, 9)
gui_prefix_ids = c(10, 11)

# attitude and trust labels
attitude_labels = c(1, 2, 3, 4, 5, 6, 7, 8, 9, 10)
trust_labels = c(1, 2, 3)

# attitude data
hand_attitude_data = list()
xbox_attitude_data = list()
gui_attitude_data = list()

for (i in 1:length(attitude_labels)) {
    pos = question_positivity[i]
    hand_id = paste0("Q", as.character(hand_prefix_ids[1]), "_", as.character(attitude_labels[i]))
    xbox_id = paste0("Q", as.character(xbox_prefix_ids[1]), "_", as.character(attitude_labels[i]))
    gui_id = paste0("Q", as.character(gui_prefix_ids[1]), "_", as.character(attitude_labels[i]))

    hand_question = qualtrics_data[3:nrow(qualtrics_data), hand_id]
     xbox_question = qualtrics_data[3:nrow(qualtrics_data), xbox_id]
    gui_question = qualtrics_data[3:nrow(qualtrics_data), gui_id]

    hand_ratings = lapply(hand_question, likert_to_number, bool_positive=pos)
    xbox_ratings = lapply(xbox_question, likert_to_number, bool_positive=pos)
    gui_ratings = lapply(gui_question, likert_to_number, bool_positive=pos)

    hand_attitude_data[[i]] = hand_ratings
    xbox_attitude_data[[i]] = xbox_ratings
    gui_attitude_data[[i]] = gui_ratings
}

# trust data
hand_trust_data = list()
xbox_trust_data = list()
gui_trust_data = list()

offset = length(attitude_labels)
for (i in 1:length(trust_labels)) {
    j = i + offset
    pos = question_positivity[j]
    hand_id = paste0("Q", as.character(hand_prefix_ids[2]), "_", as.character(trust_labels[i]))
    xbox_id = paste0("Q", as.character(xbox_prefix_ids[2]), "_", as.character(trust_labels[i]))
    gui_id = paste0("Q", as.character(gui_prefix_ids[2]), "_", as.character(trust_labels[i]))

    hand_question = qualtrics_data[3:nrow(qualtrics_data), hand_id]
    xbox_question = qualtrics_data[3:nrow(qualtrics_data), xbox_id]
    gui_question = qualtrics_data[3:nrow(qualtrics_data), gui_id]

    hand_ratings = lapply(hand_question, likert_to_number, bool_positive=TRUE)
    xbox_ratings = lapply(xbox_question, likert_to_number, bool_positive=TRUE)
    gui_ratings = lapply(gui_question, likert_to_number, bool_positive=TRUE)

    hand_trust_data[[i]] = hand_ratings
    xbox_trust_data[[i]] = xbox_ratings
    gui_trust_data[[i]] = gui_ratings
}

# summarize attitude data
hand_attitude_data = unlist(hand_attitude_data)
print(summary(hand_attitude_data))
xbox_attitude_data = unlist(xbox_attitude_data)
print(summary(xbox_attitude_data))
gui_attitude_data = unlist(gui_attitude_data)
print(summary(gui_attitude_data))

# condense trust data
hand_trust_data = unlist(hand_trust_data)
xbox_trust_data = unlist(xbox_trust_data)
gui_trust_data = unlist(gui_trust_data)

box_attitude_data = data.frame(Hand.Gestures=hand_attitude_data, Xbox=xbox_attitude_data, GUI=gui_attitude_data)
box_trust_data = data.frame(Hand.Gestures=hand_trust_data, Xbox=xbox_trust_data, GUI=gui_trust_data)

# plot
par(mfrow=c(2, 1))
boxplot(box_attitude_data, main="Technology Acceptance", xlab="System", ylab="Likert Rating", col="gray")
boxplot(box_trust_data, main="Trust", xlab="System", ylab="Likert Rating", col="gray")
