# set working directory
path_to_project <- "/Users/mlamsey/Documents/GT/Coursework/CS7633 Human-Robot Interaction/cs7633-project"
data_path <- paste(path_to_project, "analysis/data", sep = "/")
setwd(data_path)

# check if log_cleaned.csv is in hri003
dir = "hri003"
file_name = "log_cleaned.csv"
file_path <- paste(data_path, dir, sep = "/")
file_path <- paste(file_path, file_name, sep = "/")

if (!file.exists(file_path)) {
    dir = "hri002"
    file_name = "log_cleaned.csv"
    file_path <- paste(data_path, dir, sep = "/")
    file_path <- paste(file_path, file_name, sep = "/")

    log_data = read.csv(file_path)

    # split at row 7423
    log_data_1 = log_data[1:7423,]
    log_data_2 = log_data[7424:length(log_data),]

    # write first half to folder hri002
    write.table(log_data_1, file = file_path, append = FALSE, row.names = FALSE, col.names = TRUE, sep = ",")

    # write second half to folder hri003
    dir = "hri003"
    file_name = "log_cleaned.csv"
    file_path <- paste(data_path, dir, sep = "/")
    file_path <- paste(file_path, file_name, sep = "/")

    write.table(log_data_2, file = file_path, append = FALSE, row.names = FALSE, col.names = TRUE, sep = ",")
}
