# helper functions
# remove every other row of a log file if the first entry is "source"
clean_rows = function(data) {
    # remove rows where the first entry is "source"
    data = data[data$source != "source", ]
    return(data)
}

#  set working directory, load data
path_to_project <- "/Users/mlamsey/Documents/GT/Coursework/CS7633 Human-Robot Interaction/cs7633-project"
data_path <- paste(path_to_project, "analysis/data", sep = "/")
setwd(data_path)

# log directories
directory_list <- c("hri001", "hri002", "hri003", "hri004", "hri005", "hri006")

# clean all
for (i in c(1, 2, 3, 4, 5, 6)){
    # set working directory
    dir = paste(data_path, directory_list[i], sep = "/")
    setwd(dir)

    # get files that begin with log
    log_files = list.files(pattern = "^log")
    if (length(log_files) == 0) {
        print(paste("No log files found in", directory_list[i], sep = " "))
        next
    }
    # print(log_files)

    # check if a cleaned log is already in the directory
    if (length(grep("log_cleaned.csv", log_files)) > 0) {
        print(paste("log_cleaned.csv already exists in", directory_list[i], sep = " "))
        next
    }

    log_data = read.csv(log_files[1])
    log_header = "source,time_s,time_ns,action,controller_state,hand_in_frame"

    # write header to file
    write(log_header, file = "log_cleaned.csv", append = FALSE)

    # write cleaned data to file
    write.table(clean_rows(log_data), file = "log_cleaned.csv", append = TRUE, row.names = FALSE, col.names = FALSE, sep = ",")
}