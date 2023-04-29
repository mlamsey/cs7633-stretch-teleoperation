import numpy as np
import csv
import pandas as pd
from scipy.stats import ttest_ind, ttest_1samp

import matplotlib.pyplot as plt

# global vars
data_directory = "/Users/mlamsey/Documents/GT/Coursework/CS7633 Human-Robot Interaction/cs7633-project/analysis/data/"
log_file = "log_cleaned.csv"
times_file = "recorded_times_cleaned.csv"
qualitrics_file = "qualtrics.csv"
id_col = "Q1"
ease_of_use_positivity = [True, False, True, False, True, False, True, False, True, False]
trust_positivity = [True, True, True]

LIKERT5 = {
    "STRONGLY DISAGREE": 1,
    "DISAGREE": 2,
    "NEUTRAL": 3,
    "AGREE": 4,
    "STRONGLY AGREE": 5
}

USAGE = {
    "I HAVE NEVER USED IT BEFORE": 1,
    "I RARELY USE THIS TECHNOLOGY": 2,
    "I USE IT A FEW TIMES A YEAR": 3,
    "I USE IT A FEW TIMES A MONTH": 4,
    "I USE IT EVERYDAY/ALMOST EVERYDAY": 5,
}

MODALITY = {
    "GUI": 1,
    "XBOX": 2,
    "HAND": 3
}

# helpers
def likert2num(likert_df):
    likert = likert_df.to_numpy()
    likert = np.vectorize(str.upper)(likert)
    likert = np.vectorize(LIKERT5.get)(likert)
    return likert

def get_ease_of_use(ids, modality):
    qualtrics_df = pd.read_csv(data_directory + qualitrics_file)
    if modality == MODALITY["HAND"]:
        ease_of_use_base = "Q6_"
    elif modality == MODALITY["XBOX"]:
        ease_of_use_base = "Q8_"
    elif modality == MODALITY["GUI"]:
        ease_of_use_base = "Q10_"

    # extract data
    qualtrics_cols = qualtrics_df.columns
    ease_of_use_cols = [col for col in qualtrics_cols if ease_of_use_base in col]
    ease_of_use_data = qualtrics_df[ease_of_use_cols][2:]
    ease_of_use_data = likert2num(ease_of_use_data)
    
    # invert negative questions
    for i in range(len(ease_of_use_cols)):
        if not ease_of_use_positivity[i]:
            ease_of_use_data[:,i] = 6 - ease_of_use_data[:,i]

    # mean participant scores
    mean_ease_of_use = np.mean(ease_of_use_data, axis=1)
    # std_ease_of_use = np.std(ease_of_use_data, axis=1)

    return mean_ease_of_use[ids]

def get_trust(ids, modality):
    qualtrics_df = pd.read_csv(data_directory + qualitrics_file)
    if modality == MODALITY["HAND"]:
        trust_base = "Q7_"
    elif modality == MODALITY["XBOX"]:
        trust_base = "Q9_"
    elif modality == MODALITY["GUI"]:
        trust_base = "Q11_"

    qualtrics_cols = qualtrics_df.columns
    trust_cols = [col for col in qualtrics_cols if trust_base in col]

# hypotheses
def hypothesis_1():
    """
    Hypothesis 1: Gender affects performance and perceived workload
    """
    
    pass

def hypothesis_2():
    """
    Hypothesis 2: People who have played video games would find xbox
    most intuitive
    """
    qualitrics_df = pd.read_csv(data_directory + qualitrics_file)
    no_games_ratings = [1, 2, 3]
    playes_games_ratings = [4, 5]
    games_ratings = qualitrics_df["Q12_4"][2:].to_numpy()
    games_ratings = np.vectorize(str.upper)(games_ratings)
    games_ratings = np.vectorize(USAGE.get)(games_ratings)
    games_ratings = games_ratings.astype(int)
    no_games_ids = np.where(np.isin(games_ratings, no_games_ratings))[0]
    playes_games_ids = np.where(np.isin(games_ratings, playes_games_ratings))[0]
    
    data = []

    print(" ")
    print("=" * 10 + " HYPOTHESIS 2 " + "=" * 10)
    for key, value in MODALITY.items():
        no_game_eou = get_ease_of_use(no_games_ids, value)
        plays_game_eou = get_ease_of_use(playes_games_ids, value)

        data.append(no_game_eou)

        # Perform a t-test
        t_statistic, p_value = ttest_ind(no_game_eou, plays_game_eou)

        # Print the results
        print(key)
        print("means: ", np.mean(no_game_eou), np.mean(plays_game_eou))
        print("t-statistic:", t_statistic)
        print("p-value:", p_value)
        print(" ")

    # plot
    plt.figure()
    a = plt.axes()
    a.boxplot(data, labels=MODALITY.keys())
    a.set_title("Ease of Use with Prior Gaming Experience")
    a.set_ylabel("Ease of Use")
    a.set_xlabel("Modality")
    a.set_xticklabels(MODALITY.keys())
    a.set_ylim([1, 5])
    plt.savefig("h2_ease_of_use_games.png")

def hypothesis_3():
    """
    Hypothesis 3: People without prior gaming experience would find GUI
    to require least effort
    """
    qualitrics_df = pd.read_csv(data_directory + qualitrics_file)
    no_games_ratings = [1, 2, 3]
    playes_games_ratings = [4, 5]
    games_ratings = qualitrics_df["Q12_4"][2:].to_numpy()
    games_ratings = np.vectorize(str.upper)(games_ratings)
    games_ratings = np.vectorize(USAGE.get)(games_ratings)
    games_ratings = games_ratings.astype(int)
    no_games_ids = np.where(np.isin(games_ratings, no_games_ratings))[0]
    playes_games_ids = np.where(np.isin(games_ratings, playes_games_ratings))[0]

    data = {}
    means = {}

    for key, value in MODALITY.items():
        no_game_eou = get_ease_of_use(no_games_ids, value)
        plays_game_eou = get_ease_of_use(playes_games_ids, value)

        means[key] = [np.mean(no_game_eou), np.mean(plays_game_eou)]
        data[key] = [no_game_eou, plays_game_eou]

    gui_mean = means["GUI"][0]
    hand_data = data["HAND"][0]
    xbox_data = data["XBOX"][0]

    print(" ")
    print("=" * 10 + " HYPOTHESIS 3 " + "=" * 10)
    t_statistic, p_value = ttest_1samp(hand_data, gui_mean, alternative="less")
    print("HAND")
    print("t-statistic:", t_statistic)
    print("p-value:", p_value)

    t_statistic, p_value = ttest_1samp(xbox_data, gui_mean, alternative="less")
    print("XBOX")
    print("t-statistic:", t_statistic)
    print("p-value:", p_value)

    # plot
    plt.figure()
    a = plt.axes()
    a.plot([0, 3], [gui_mean, gui_mean], color="red", label="GUI Mean")
    a.boxplot([hand_data, xbox_data], labels=["Hand", "Xbox"])
    a.set_title("Ease of Use without Prior Gaming Experience")
    a.set_ylabel("Ease of Use")
    a.set_xlabel("Modality")
    a.set_xticklabels(["Hand", "Xbox"])
    a.set_ylim([1, 5])
    a.legend(["GUI Mean", "Hand", "Xbox"])
    # plt.show()
    plt.savefig("h3_ease_of_use_no_games.png")


def hypothesis_4():
    """
    Hypothesis 4: People who are visual learners would find the GUI
    method to require the least effort, compared to those with other
    learning styles
    """
    qualitrics_df = pd.read_csv(data_directory + qualitrics_file)
    learning_style = qualitrics_df["Q5"][2:].to_numpy()
    learning_style = np.vectorize(str.upper)(learning_style)
    visual = [i for i in range(len(learning_style)) if "VISUAL" in learning_style[i]]
    not_visual = [i for i in range(len(learning_style)) if "VISUAL" not in learning_style[i]]
    # print("visual")
    # print(visual)
    # print(not_visual)

    data = {}
    means = {}

    for key, value in MODALITY.items():
        visual_eou = get_ease_of_use(visual, value)
        not_visual_eou = get_ease_of_use(not_visual, value)

        means[key] = (np.mean(visual_eou), np.mean(not_visual_eou))
        data[key] = (visual_eou, not_visual_eou)

    # one sided t tests
    gui_mean = means["GUI"][0]
    hand_data = data["HAND"][0]
    xbox_data = data["XBOX"][0]

    print(" ")
    print("=" * 10 + " HYPOTHESIS 4 " + "=" * 10)
    t_statistic, p_value = ttest_1samp(hand_data, gui_mean, alternative="less")
    print("hand vs gui")
    print("t-statistic:", t_statistic)
    print("p-value:", p_value)

    t_statistic, p_value = ttest_1samp(xbox_data, gui_mean, alternative="less")
    print("xbox vs gui")
    print("t-statistic:", t_statistic)
    print("p-value:", p_value)

    # plot
    plt.figure()
    a = plt.axes()
    a.set_title("Ease of Use for Visual Learners")
    a.set_ylabel("Ease of Use")
    a.set_xlabel("Modality")
    a.plot([0, 3], [gui_mean, gui_mean], color="red", label="GUI")
    a.boxplot([hand_data, xbox_data])
    a.set_xticklabels(["Hand", "Xbox"])
    a.set_xlim(0, 3)
    a.set_ylim(1, 5 )
    plt.legend()
    plt.savefig("h4_ease_of_use_visual.png")

def hypothesis_5():
    """
    Hypothesis 5: People who are kinesthetic learners would find the
    hand gesture method to require the least effort, compared to those
    with other learning styles
    """
    # do the same as hypothesis 4 but with kinesthetic
    qualitrics_df = pd.read_csv(data_directory + qualitrics_file)
    learning_style = qualitrics_df["Q5"][2:].to_numpy()
    learning_style = np.vectorize(str.upper)(learning_style)
    kinesthetic = [i for i in range(len(learning_style)) if "KINESTHETIC" in learning_style[i]]
    not_kinesthetic = [i for i in range(len(learning_style)) if "KINESTHETIC" not in learning_style[i]]

    data = {}
    means = {}

    for key, value in MODALITY.items():
        kinesthetic_eou = get_ease_of_use(kinesthetic, value)
        not_kinesthetic_eou = get_ease_of_use(not_kinesthetic, value)

        means[key] = (np.mean(kinesthetic_eou), np.mean(not_kinesthetic_eou))
        data[key] = (kinesthetic_eou, not_kinesthetic_eou)
    
    # one sided t tests
    hand_mean = means["HAND"][0]
    gui_data = data["GUI"][0]
    xbox_data = data["XBOX"][0]

    print(" ")
    print("=" * 10 + " HYPOTHESIS 5 " + "=" * 10)
    t_statistic, p_value = ttest_1samp(gui_data, hand_mean, alternative="greater")
    print("gui vs hand")
    print("t-statistic:", t_statistic)
    print("p-value:", p_value)

    t_statistic, p_value = ttest_1samp(xbox_data, hand_mean, alternative="greater")
    print("xbox vs hand")
    print("t-statistic:", t_statistic)
    print("p-value:", p_value)

    # plot
    a = plt.axes()
    a.plot([0, 3], [hand_mean, hand_mean], color="red", label="hand mean")
    a.boxplot([gui_data, xbox_data])
    a.set_ylim(1, 5)
    a.set_xticklabels(["GUI", "XBOX"])
    a.set_title("Ease of Use for Kinesthetic Learners")
    a.legend(["Hand Mean","GUI", "XBOX"])
    a.set_ylabel("Ease of Use")
    a.set_xlabel("Modality")
    a.set_xlim(0, 3)
    # plt.grid()

    plt.savefig("h5_ease_of_use_kinesthetic.png")

if __name__ == '__main__':

    hypothesis_2()
    hypothesis_3()
    hypothesis_4()
    hypothesis_5()

