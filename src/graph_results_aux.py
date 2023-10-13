# -*- coding: utf-8 -*-
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# Read CSV
file_path = '/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/results_aux.csv'
df = pd.read_csv(file_path, encoding='utf-8')

# Remove rows with field name
df = df[df["%time"] != "%time"]

# Convert to numbers and round columns
numeric_columns = [
    "field.ALGOR", "field.Stop_AND_Wait", "field.robot_stuck", "field.CRIT_DIST",
    "field.AZIMUTH_ERROR", "field.CA_DES_VEL", "field.avoided_collisions",
    "field.fatal_collisions", "field.navigation_time", "field.CA_time",
    "field.distance_travelled", "field.optimal_time", "field.optimal_distance_travelled"
]
df[numeric_columns] = df[numeric_columns].apply(pd.to_numeric, errors='coerce')
df[numeric_columns] = df[numeric_columns].round(2)

# Order by robot name
robot_names = sorted(df["field.robot_name"].unique())

#df = df[(df["field.fatal_collisions"] != 0)] 

#plt.title('Robot {}'.format(robot))

# Iterate through robots and create a separate figure for each
for robot in robot_names:
    # Create a new figure for each robot
    plt.figure(figsize=(12, 12), num = 'Robot {}'.format(robot))
    
    #plt.subplots_adjust(hspace=0.3, wspace=0.05)
    
    group_df = df[df["field.robot_name"] == robot]
    
    if not group_df.empty:
        # Create the first subplot for CRIT_DIST
        plt.subplot(3, 4, 1)
        sns.boxplot(x="field.CRIT_DIST", y="field.avoided_collisions", data=group_df)
        plt.xlabel("CRIT_DIST")
        plt.ylabel("Colisiones Evitadas")
        plt.title('Colisiones Evitadas vs CRIT_DIST')

        # Create the first subplot for CRIT_DIST
        plt.subplot(3, 4, 2)
        sns.boxplot(x="field.CRIT_DIST", y="field.navigation_time", data=group_df)
        plt.xlabel("CRIT_DIST")
        plt.ylabel("navigation_time")
        plt.title('navigation_time vs CRIT_DIST')

        # Create the first subplot for CRIT_DIST
        plt.subplot(3, 4, 3)
        sns.boxplot(x="field.CRIT_DIST", y="field.CA_time", data=group_df)
        plt.xlabel("CRIT_DIST")
        plt.ylabel("CA_time")
        plt.title('CA_time vs CRIT_DIST')

        # Create the first subplot for CRIT_DIST
        plt.subplot(3, 4, 4)
        sns.boxplot(x="field.CRIT_DIST", y="field.distance_travelled", data=group_df)
        plt.xlabel("CRIT_DIST")
        plt.ylabel("distance_travelled")
        plt.title('distance_travelled vs CRIT_DIST')

        # Create the second subplot for AZIMUTH_ERROR
        plt.subplot(3, 4, 5)
        sns.boxplot(x="field.AZIMUTH_ERROR", y="field.avoided_collisions", data=group_df)
        plt.xlabel("AZIMUTH_ERROR")
        plt.ylabel("Colisiones Evitadas")
        plt.title('Colisiones Evitadas vs AZIMUTH_ERROR')

        # Create the second subplot for AZIMUTH_ERROR
        plt.subplot(3, 4, 6)
        sns.boxplot(x="field.AZIMUTH_ERROR", y="field.navigation_time", data=group_df)
        plt.xlabel("AZIMUTH_ERROR")
        plt.ylabel("navigation_time")
        plt.title('navigation_time vs AZIMUTH_ERROR')

        # Create the second subplot for AZIMUTH_ERROR
        plt.subplot(3, 4, 7)
        sns.boxplot(x="field.AZIMUTH_ERROR", y="field.CA_time", data=group_df)
        plt.xlabel("AZIMUTH_ERROR")
        plt.ylabel("CA_time")
        plt.title('CA_time vs AZIMUTH_ERROR')

        # Create the second subplot for AZIMUTH_ERROR
        plt.subplot(3, 4, 8)
        sns.boxplot(x="field.AZIMUTH_ERROR", y="field.distance_travelled", data=group_df)
        plt.xlabel("AZIMUTH_ERROR")
        plt.ylabel("distance_travelled")
        plt.title('distance_travelled vs AZIMUTH_ERROR')

        # Create the second subplot for AZIMUTH_ERROR
        plt.subplot(3, 4, 9)
        sns.boxplot(x="field.CA_DES_VEL", y="field.avoided_collisions", data=group_df)
        plt.xlabel("CA_DES_VEL")
        plt.ylabel("Colisiones Evitadas")
        plt.title('Colisiones Evitadas vs CA_DES_VEL')

        # Create the second subplot for AZIMUTH_ERROR
        plt.subplot(3, 4, 10)
        sns.boxplot(x="field.CA_DES_VEL", y="field.navigation_time", data=group_df)
        plt.xlabel("CA_DES_VEL")
        plt.ylabel("navigation_time")
        plt.title('navigation_time vs CA_DES_VEL')

        # Create the second subplot for AZIMUTH_ERROR
        plt.subplot(3, 4, 11)
        sns.boxplot(x="field.CA_DES_VEL", y="field.CA_time", data=group_df)
        plt.xlabel("CA_DES_VEL")
        plt.ylabel("CA_time")
        plt.title('CA_time vs CA_DES_VEL')

        # Create the second subplot for AZIMUTH_ERROR
        plt.subplot(3, 4, 12)
        sns.boxplot(x="field.CA_DES_VEL", y="field.distance_travelled", data=group_df)
        plt.xlabel("CA_DES_VEL")
        plt.ylabel("distance_travelled")
        plt.title('distance_travelled vs CA_DES_VEL')

    # Display the plot for the current robot
    plt.tight_layout()
    plt.show()

