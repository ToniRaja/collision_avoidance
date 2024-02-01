# -*- coding: utf-8 -*-
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

file_path = '/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/parametrization_results.csv'

df = pd.read_csv(file_path, header=None, encoding = "utf-8")

if '%time' in df.iloc[0].values:
    df = df.drop(columns=df.columns[0])

if 'field.robot_name' in df.iloc[0].values:
    df = df.drop(columns=df.columns[0])

print(df)

df.columns = range(len(df.columns))

result_columns = ['ALGOR', 'STOP_AND_WAIT', 'CRIT_DIST', 'DES_VEL', 'ORI_ERROR', 'W1', 'W2',
                  'ROT_VEL', 'K_ROT_MAX', 'K_ROT_MIN', 'time_step','time_horizon', 'sphere_radius',
                  'robot_stuck', 'avoided_collisions', 'fatal_collisions',
                  'navigation_time', 'CA_time', 'distance_travelled', 'optimal_time', 'optimal_distance_travelled']

result_df = pd.DataFrame(columns=result_columns)

current_group = None  
group_data = []       

for index, row in df.iterrows():
    if row.iloc[0] == 'field.ALGOR':
        if current_group:
            current_group[result_columns.index('ALGOR')] = current_algor
            current_group[result_columns.index('STOP_AND_WAIT')] = current_saw
            current_group[result_columns.index('W1')] = current_w1
            current_group[result_columns.index('W2')] = current_w2
            current_group[result_columns.index('time_step')] = current_ts
            current_group[result_columns.index('time_horizon')] = current_th
            current_group[result_columns.index('sphere_radius')] = current_sr
            current_group[result_columns.index('CRIT_DIST')] = current_CD
            current_group[result_columns.index('DES_VEL')] = current_DV
            current_group[result_columns.index('ROT_VEL')] = current_RV
            current_group[result_columns.index('K_ROT_MAX')] = current_KRMAX
            current_group[result_columns.index('K_ROT_MIN')] = current_KRMIN
            current_group[result_columns.index('ORI_ERROR')] = current_AE
            current_group[result_columns.index('robot_stuck')] = sum(int(data[result_columns.index('robot_stuck')]) for data in group_data)
            current_group[result_columns.index('avoided_collisions')] = sum(int(data[result_columns.index('avoided_collisions')]) for data in group_data)
            current_group[result_columns.index('fatal_collisions')] = int(sum(float(data[result_columns.index('fatal_collisions')]) for data in group_data))
            current_group[result_columns.index('navigation_time')] = round(sum(float(data[result_columns.index('navigation_time')]) for data in group_data) / len(group_data), 2)
            current_group[result_columns.index('CA_time')] = round(sum(float(data[result_columns.index('CA_time')]) for data in group_data) / len(group_data), 2)
            current_group[result_columns.index('distance_travelled')] = round(sum(float(data[result_columns.index('distance_travelled')]) for data in group_data) / len(group_data), 2)
            current_group[result_columns.index('optimal_time')] = round(sum(float(data[result_columns.index('optimal_time')]) for data in group_data) / len(group_data), 2)
            current_group[result_columns.index('optimal_distance_travelled')] = round(sum(float(data[result_columns.index('optimal_distance_travelled')]) for data in group_data) / len(group_data), 2)
            result_df = result_df.append(pd.Series(current_group, index=result_columns), ignore_index=True)

        current_group = row.values.tolist()
        group_data = []
    
        current_algor = df.iloc[index + 1, 0] if index + 1 < len(df) else None
        current_saw = df.iloc[index + 1, 1] if index + 1 < len(df) else None
        current_w1 = df.iloc[index + 1, 2] if index + 1 < len(df) else None
        current_w2 = df.iloc[index + 1, 3] if index + 1 < len(df) else None
        current_ts = round(float(df.iloc[index + 1, 4]), 1) if index + 1 < len(df) else None
        current_th = round(float(df.iloc[index + 1, 5]), 1) if index + 1 < len(df) else None
        current_sr = round(float(df.iloc[index + 1, 6]), 1) if index + 1 < len(df) else None
        current_CD = round(float(df.iloc[index + 1, 7]), 1) if index + 1 < len(df) else None
        current_DV = round(float(df.iloc[index + 1, 8]), 1) if index + 1 < len(df) else None
        current_RV = round(float(df.iloc[index + 1, 9]), 1) if index + 1 < len(df) else None
        current_KRMAX = round(float(df.iloc[index + 1, 10]), 1) if index + 1 < len(df) else None
        current_KRMIN = round(float(df.iloc[index + 1, 11]), 1) if index + 1 < len(df) else None
        current_AE = round(float(df.iloc[index + 1, 12]), 1) if index + 1 < len(df) else None

    elif current_group:
        group_data.append(row.values.tolist())

current_group[result_columns.index('ALGOR')] = current_algor
current_group[result_columns.index('STOP_AND_WAIT')] = current_saw
current_group[result_columns.index('W1')] = current_w1
current_group[result_columns.index('W2')] = current_w2
current_group[result_columns.index('time_step')] = current_ts
current_group[result_columns.index('time_horizon')] = current_th
current_group[result_columns.index('sphere_radius')] = current_sr
current_group[result_columns.index('CRIT_DIST')] = current_CD
current_group[result_columns.index('DES_VEL')] = current_DV
current_group[result_columns.index('ROT_VEL')] = current_RV
current_group[result_columns.index('K_ROT_MAX')] = current_KRMAX
current_group[result_columns.index('K_ROT_MIN')] = current_KRMIN
current_group[result_columns.index('ORI_ERROR')] = current_AE
current_group[result_columns.index('robot_stuck')] = sum(int(data[result_columns.index('robot_stuck')]) for data in group_data)
current_group[result_columns.index('avoided_collisions')] = sum(int(data[result_columns.index('avoided_collisions')]) for data in group_data)
current_group[result_columns.index('fatal_collisions')] = int(sum(float(data[result_columns.index('fatal_collisions')]) for data in group_data))
current_group[result_columns.index('navigation_time')] = round(sum(float(data[result_columns.index('navigation_time')]) for data in group_data) / len(group_data), 2)
current_group[result_columns.index('CA_time')] = round(sum(float(data[result_columns.index('CA_time')]) for data in group_data) / len(group_data), 2)
current_group[result_columns.index('distance_travelled')] = round(sum(float(data[result_columns.index('distance_travelled')]) for data in group_data) / len(group_data), 2)
current_group[result_columns.index('optimal_time')] = round(sum(float(data[result_columns.index('optimal_time')]) for data in group_data) / len(group_data), 2)
current_group[result_columns.index('optimal_distance_travelled')] = round(sum(float(data[result_columns.index('optimal_distance_travelled')]) for data in group_data) / len(group_data), 2)
result_df = result_df.append(pd.Series(current_group, index=result_columns), ignore_index=True)

result_df['CRIT_DIST'] = result_df['CRIT_DIST'].astype(float)
result_df['avoided_collisions'] = result_df['avoided_collisions'].astype(int)
result_df['fatal_collisions'] = result_df['fatal_collisions'].astype(int)
result_df['robot_stuck'] = result_df['robot_stuck'].astype(int)

result_df = result_df.reset_index(drop=True)

PF_df = result_df[result_df['ALGOR'] == '1']
ORCA_df = result_df[result_df['ALGOR'] == '2']

PF_df = PF_df.reset_index(drop=True)
ORCA_df = ORCA_df.reset_index(drop=True)

print("")
print("Potential Fields full dataframe")
print("")
print(PF_df)
print("")
print("ORCA full dataframe")
print("")
print(ORCA_df)

PF_initial_num_sim = len(PF_df)
ORCA_initial_num_sim = len(ORCA_df)

PF_df_1 = PF_df[(PF_df["fatal_collisions"] == 0)]
ORCA_df_1 = ORCA_df[(ORCA_df["fatal_collisions"] == 0)]

PF_df_1 = PF_df_1.reset_index(drop=True)
ORCA_df_1 = ORCA_df_1.reset_index(drop=True)

print("")
print("Potential Fields dataframe without unsuccesfull simulations")
print("")
print(PF_df_1)
print("")
print("ORCA dataframe without unsuccesfull simulations")
print("")
print(ORCA_df_1)

PF_df = PF_df.groupby(['ALGOR', 'STOP_AND_WAIT', 'CRIT_DIST', 'DES_VEL', 'ORI_ERROR', 'W1', 'W2',
        'ROT_VEL', 'K_ROT_MAX', 'K_ROT_MIN', 'time_step','time_horizon', 'sphere_radius']).filter(lambda x: (x['fatal_collisions'] == 0).all())
ORCA_df = ORCA_df.groupby(['ALGOR', 'STOP_AND_WAIT', 'CRIT_DIST', 'DES_VEL', 'ORI_ERROR', 'W1', 'W2',
        'ROT_VEL', 'K_ROT_MAX', 'K_ROT_MIN', 'time_step','time_horizon', 'sphere_radius']).filter(lambda x: (x['fatal_collisions'] == 0).all())

PF_df = PF_df.reset_index(drop=True)
ORCA_df = ORCA_df.reset_index(drop=True)

PF_num_sim = len(PF_df)
ORCA_num_sim = len(ORCA_df)

print("")
print("Potential Fields dataframe without unsuccesfull parameter combinations")
print("")
print(PF_df)
print("")
print("ORCA dataframe without unsuccesfull parameter combinations")
print("")
print(ORCA_df)

metrics = ['avoided_collisions', 'navigation_time', 'CA_time', 'distance_travelled']
columns = ['ALGOR','CRIT_DIST', 'ORI_ERROR', 'W2', ]

PF_mean = PF_df.groupby(columns)[metrics].mean().add_prefix('Mean_')
PF_min = PF_df.groupby(columns)[metrics].min().add_prefix('Minimum_')
PF_max = PF_df.groupby(columns)[metrics].max().add_prefix('Maximum_')
PF_summary = pd.concat([PF_mean, PF_min, PF_max], axis=1)
PF_summary = PF_summary.round(2)

columns = ['ALGOR','CRIT_DIST', 'ORI_ERROR', 'time_step', 'time_horizon']

ORCA_mean = ORCA_df.groupby(columns)[metrics].mean().add_prefix('Mean_')
ORCA_min = ORCA_df.groupby(columns)[metrics].min().add_prefix('Minimum_')
ORCA_max = ORCA_df.groupby(columns)[metrics].max().add_prefix('Maximum_')
ORCA_summary = pd.concat([ORCA_mean, ORCA_min, ORCA_max], axis=1)
ORCA_summary = ORCA_summary.round(2)

PF_summary.reset_index(inplace=True)
ORCA_summary.reset_index(inplace=True)

PF_summary.to_csv('/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/PF_summary.csv', index=False)
ORCA_summary.to_csv('/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/ORCA_summary.csv', index=False)

print("")
print("Potential Fields summary")
print("")
print(PF_summary)
print("")
print("ORCA summary")
print("")
print(ORCA_summary)

PF_df = PF_df.groupby(['CRIT_DIST', 'ORI_ERROR', 'W2', 'optimal_time', 'optimal_distance_travelled']).mean()[metrics]
ORCA_df = ORCA_df.groupby(['CRIT_DIST', 'ORI_ERROR', 'time_step', 'time_horizon', 'optimal_time', 'optimal_distance_travelled']).mean()[metrics]

PF_df.reset_index(inplace=True)
ORCA_df.reset_index(inplace=True)

print("")
print("Potential Fields dataframe by parameter combination")
print("")
print(PF_df)
print("")
print("ORCA dataframe by parameter combination")
print("")
print(ORCA_df)

plt.figure(figsize=(12, 12), num='Potential Fields simulations ({} simulations out of {} with no collision)'.format(PF_num_sim, PF_initial_num_sim))
plt.suptitle('APF simulations ({} simulations out of {} with no collision)'.format(PF_num_sim, PF_initial_num_sim), fontsize=13)
plot_variables = [
    ("CRIT_DIST", "avoided_collisions", "Avoided Collisions vs\n CRIT_DIST"),
    ("CRIT_DIST", "navigation_time", "Navigation Time vs\n CRIT_DIST"),
    ("CRIT_DIST", "CA_time", "Collision Avoidance Time vs\n CRIT_DIST"),
    ("CRIT_DIST", "distance_travelled", "Distance Traveled vs\n CRIT_DIST"),
    ("ORI_ERROR", "avoided_collisions", "Avoided Collisions vs\n ORI_ERROR"),
    ("ORI_ERROR", "navigation_time", "Navigation Time vs\n ORI_ERROR"),
    ("ORI_ERROR", "CA_time", "Collision Avoidance Time vs\n ORI_ERROR"),
    ("ORI_ERROR", "distance_travelled", "Distance Traveled vs\n ORI_ERROR"),
    ("W2", "avoided_collisions", "Avoided Collisions vs\n W2"),
    ("W2", "navigation_time", "Navigation Time vs\n W2"),
    ("W2", "CA_time", "Collision Avoidance Time vs\n W2"),
    ("W2", "distance_travelled", "Distance Traveled vs\n W2"),
]

optimal_time_mean = 10 * PF_df['optimal_time'].mean()

for i, (x_var, y_var, title) in enumerate(plot_variables, start=1):
    if x_var in PF_df.columns and y_var in PF_df.columns:
        plt.subplot(3, 4, i)
        sns.boxplot(x=x_var, y=y_var, data=PF_df)
        plt.xlabel(x_var)
        plt.ylabel(y_var)
        plt.title(title)

        if y_var == "avoided_collisions":
            plt.ylabel("Avoided Collisions")
        elif y_var == "navigation_time":
            plt.ylabel("Navigation Time")
        elif y_var == "CA_time":
            plt.ylabel("Collision Avoidance Time")
        elif y_var == "distance_travelled":
            plt.ylabel("Distance Traveled")
        
        if y_var in ["navigation_time", "CA_time"]:
            plt.axhline(y=optimal_time_mean, color='red', linewidth=2, linestyle='solid')
            
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.savefig('/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/parametrization_APF.png')
plt.show()

plt.figure(figsize=(12, 12), num='ORCA simulations ({} simulations out of {} with no collision)'.format(ORCA_num_sim, ORCA_initial_num_sim))
plt.suptitle('ORCA simulations ({} simulations out of {} with no collision)'.format(ORCA_num_sim, ORCA_initial_num_sim), fontsize=13)
plot_variables = [
    ("CRIT_DIST", "avoided_collisions", "Avoided Collisions vs\n CRIT_DIST"),
    ("CRIT_DIST", "navigation_time", "Navigation Time vs\n CRIT_DIST"),
    ("CRIT_DIST", "CA_time", "Collision Avoidance Time vs\n CRIT_DIST"),
    ("CRIT_DIST", "distance_travelled", "Distance Traveled vs\n CRIT_DIST"),
    ("ORI_ERROR", "avoided_collisions", "Avoided Collisions vs\n ORI_ERROR"),
    ("ORI_ERROR", "navigation_time", "Navigation Time vs\n ORI_ERROR"),
    ("ORI_ERROR", "CA_time", "Collision Avoidance Time vs\n ORI_ERROR"),
    ("ORI_ERROR", "distance_travelled", "Distance Traveled vs\n ORI_ERROR"),
]

optimal_time_mean = 10 * ORCA_df['optimal_time'].mean()

for i, (x_var, y_var, title) in enumerate(plot_variables, start=1):
    if x_var in PF_df.columns and y_var in ORCA_df.columns:
        plt.subplot(3, 4, i)
        sns.boxplot(x=x_var, y=y_var, data=ORCA_df)
        plt.xlabel(x_var)
        plt.ylabel(y_var)
        plt.title(title)

        if y_var == "avoided_collisions":
            plt.ylabel("Avoided Collisions")
        elif y_var == "navigation_time":
            plt.ylabel("Navigation Time")
        elif y_var == "CA_time":
            plt.ylabel("Collision Avoidance Time")
        elif y_var == "distance_travelled":
            plt.ylabel("Distance Traveled")
        
        if y_var in ["navigation_time", "CA_time"]:
            plt.axhline(y=optimal_time_mean, color='red', linewidth=2, linestyle='solid')
            
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.savefig('/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/parametrization_ORCA_1.png')

plt.show()

plt.figure(figsize=(12, 12), num='ORCA simulations ({} simulations out of {} with no collision)'.format(ORCA_num_sim, ORCA_initial_num_sim))
plt.suptitle('ORCA simulations ({} simulations out of {} with no collision)'.format(ORCA_num_sim, ORCA_initial_num_sim), fontsize=13)
plot_variables = [
    ("time_step", "avoided_collisions", "Avoided Collisions vs\n time_step"),
    ("time_step", "navigation_time", "Navigation Time vs\n time_step"),
    ("time_step", "CA_time", "Collision Avoidance Time vs\n time_step"),
    ("time_step", "distance_travelled", "Distance Traveled vs\n time_step"),
    ("time_horizon", "avoided_collisions", "Avoided Collisions vs\n time_horizon"),
    ("time_horizon", "navigation_time", "Navigation Time vs\n time_horizon"),
    ("time_horizon", "CA_time", "Collision Avoidance Time vs\n time_horizon"),
    ("time_horizon", "distance_travelled", "Distance Traveled vs\n time_horizon"),
]

for i, (x_var, y_var, title) in enumerate(plot_variables, start=1):
    if x_var in ORCA_df.columns and y_var in ORCA_df.columns:
        plt.subplot(3, 4, i)
        sns.boxplot(x=x_var, y=y_var, data=ORCA_df)
        plt.xlabel(x_var)
        plt.ylabel(y_var)
        plt.title(title)

        if y_var == "avoided_collisions":
            plt.ylabel("Avoided Collisions")
        elif y_var == "navigation_time":
            plt.ylabel("Navigation Time")
        elif y_var == "CA_time":
            plt.ylabel("Collision Avoidance Time")
        elif y_var == "distance_travelled":
            plt.ylabel("Distance Traveled")
        
        if y_var in ["navigation_time", "CA_time"]:
            plt.axhline(y=optimal_time_mean, color='red', linewidth=2, linestyle='solid')
            
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.savefig('/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/parametrization_ORCA_2.png')

plt.show()

PF_df = PF_df.sort_values(by=["navigation_time", "distance_travelled", "CA_time", "avoided_collisions"], ascending=[True, True, True, True])
PF_best_result = PF_df.iloc[0]
print("")
print("Potential Fields best simulation")
print("")
print(PF_best_result)

ORCA_df = ORCA_df.sort_values(by=["navigation_time", "distance_travelled", "CA_time", "avoided_collisions"], ascending=[True, True, True, True])
ORCA_best_result = ORCA_df.iloc[0]
print("")
print("ORCA2-3D best simulation")
print("")    
print(ORCA_best_result)

file_path = '/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/comparative_4robots.csv'

df = pd.read_csv(file_path, header=None, encoding = "utf-8")

if '%time' in df.iloc[0].values:
    df = df.drop(columns=df.columns[0])

result_columns = ['robot_name', 'ALGOR', 'STOP_AND_WAIT', 'W1', 'W2', 'time_step', 'time_horizon',
                  'sphere_radius', 'CRIT_DIST', 'DES_VEL', 'CA_ROT_VEL',  'K_ROT_MAX',
                  'K_ROT_MIN', 'ORI_ERROR', 'robot_stuck', 'avoided_collisions',  'fatal_collisions', 
                  'navigation_time', 'CA_time', 'distance_travelled', 'optimal_time', 'optimal_distance_travelled']

result_df = pd.DataFrame(columns=result_columns)

filtered_df = df[df.iloc[:, 0].str.startswith('field.') == False].copy()

filtered_df.reset_index(drop=True, inplace=True)

result_df['robot_name'] = filtered_df.iloc[:, 0]

result_df[result_columns[1:]] = filtered_df.iloc[:, 1:]

print(result_df)

result_df['optimal_time'] = pd.to_numeric(result_df['optimal_time'], errors='coerce')
optimal_time_mean = 10 * result_df['optimal_time'].mean()

metrics = ['navigation_time', 'CA_time', 'avoided_collisions', 'distance_travelled']

simulation_types = {
    ('1', '1'): 'APF with SAW',
    ('1', '0'): 'APF',
    ('2', '1'): 'ORCA with SAW',
    ('2', '0'): 'ORCA'
}

agg_data = pd.DataFrame(columns=['Robot Name', 'Metric', 'Simulation Type', 'Value'])

for metric in metrics:
    for (algo, sw), sim_type in simulation_types.items():
        filtered_data = result_df[(result_df['ALGOR'] == algo) & (result_df['STOP_AND_WAIT'] == sw)].copy()
        
        if not filtered_data.empty:
            rounded_values = filtered_data[metric].astype(float).round(2)
            filtered_data['Metric'] = metric
            filtered_data['Simulation Type'] = sim_type
            filtered_data['Value'] = rounded_values
            agg_data = pd.concat([agg_data, filtered_data[['robot_name', 'Metric', 'Simulation Type', 'Value']]], sort=False)
            
sns.set_style("whitegrid")
sns.set_palette("husl")

ordered_robot_names = sorted(agg_data['robot_name'].unique(), key=lambda x: int(x.split('robot')[1]))

fig, axes = plt.subplots(2, 2, figsize=(12, 10))

fatal_collisions_counts = {
    'APF': result_df[(result_df['ALGOR'] == '1') & (result_df['STOP_AND_WAIT'] == '0')]['fatal_collisions'].astype(int).sum(),
    'APF with SAW': result_df[(result_df['ALGOR'] == '1') & (result_df['STOP_AND_WAIT'] == '1')]['fatal_collisions'].astype(int).sum(),
    'ORCA': result_df[(result_df['ALGOR'] == '2') & (result_df['STOP_AND_WAIT'] == '0')]['fatal_collisions'].astype(int).sum(),
    'ORCA with SAW': result_df[(result_df['ALGOR'] == '2') & (result_df['STOP_AND_WAIT'] == '1')]['fatal_collisions'].astype(int).sum()
}

fatal_collisions_subtitle = 'Not Avoided Collisions: ' + \
    'APF ({}) '.format(fatal_collisions_counts['APF']) + \
    'APF with SAW ({}) '.format(fatal_collisions_counts['APF with SAW']) + \
    'ORCA ({})  '.format(fatal_collisions_counts['ORCA']) + \
    'ORCA with SAW ({}) '.format(fatal_collisions_counts['ORCA with SAW'])

fig.suptitle('Comparison of Metrics by Robot and Simulation Type\n{}'.format(fatal_collisions_subtitle), fontsize=13)


for i, metric in enumerate(metrics):
    row = i // 2
    col = i % 2
    ax = axes[row, col]
    
    if metric in ['navigation_time', 'CA_time']:
        ax.axhline(y=optimal_time_mean, color='red', linewidth=2, linestyle='solid', label='Time Limit')

    sns.barplot(data=agg_data[agg_data['Metric'] == metric], x='robot_name', y='Value', hue='Simulation Type', ax=ax,
                order=ordered_robot_names, dodge=True)
    
    if metric == "navigation_time":
        ax.set_title('Navigation Time')
        ax.set_ylabel("Navigation Time")
    elif metric == "CA_time":
        ax.set_title('Collision Avoidance Time')
        ax.set_ylabel("Collision Avoidance Time")
    elif metric == "avoided_collisions":
        ax.set_title('Avoided Collisions')
        ax.set_ylabel("Avoided Collisions")
    else:
        ax.set_title('Distance Traveled')
        ax.set_ylabel("Distance Traveled")
    ax.set_xlabel('Robot Name')
    ax.legend(loc='upper left', prop={'size': 9})
    
    ax.set_yticklabels(['{:,.0f}'.format(x) for x in ax.get_yticks()])

last_ax = axes[-1, -1]
handles, labels = last_ax.get_legend_handles_labels()
last_ax.legend(handles=handles, labels=labels, loc='upper right', prop={'size': 9})

plt.tight_layout(rect=[0, 0.03, 1, 0.95])

plt.savefig('/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/HD_4AUV.png')

plt.show()

file_path = '/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/comparative_8robots.csv'

df = pd.read_csv(file_path, header=None, encoding = "utf-8")

if '%time' in df.iloc[0].values:
    df = df.drop(columns=df.columns[0])

result_columns = ['robot_name', 'ALGOR', 'STOP_AND_WAIT', 'W1', 'W2', 'time_step', 'time_horizon',
                  'sphere_radius', 'CRIT_DIST', 'DES_VEL', 'CA_ROT_VEL',  'K_ROT_MAX',
                  'K_ROT_MIN', 'ORI_ERROR', 'robot_stuck', 'avoided_collisions',  'fatal_collisions', 
                  'navigation_time', 'CA_time', 'distance_travelled', 'optimal_time', 'optimal_distance_travelled']

result_df = pd.DataFrame(columns=result_columns)

filtered_df = df[df.iloc[:, 0].str.startswith('field.') == False].copy()

filtered_df.reset_index(drop=True, inplace=True)

result_df['robot_name'] = filtered_df.iloc[:, 0]

result_df[result_columns[1:]] = filtered_df.iloc[:, 1:]

print(result_df)

result_df['optimal_time'] = pd.to_numeric(result_df['optimal_time'], errors='coerce')
optimal_time_mean = 10 * result_df['optimal_time'].mean()

metrics = ['navigation_time', 'CA_time', 'avoided_collisions', 'distance_travelled']

simulation_types = {
    ('1', '1'): 'APF with SAW',
    ('1', '0'): 'APF',
    ('2', '1'): 'ORCA with SAW',
    ('2', '0'): 'ORCA'
}

agg_data = pd.DataFrame(columns=['Robot Name', 'Metric', 'Simulation Type', 'Value'])

for metric in metrics:
    for (algo, sw), sim_type in simulation_types.items():
        filtered_data = result_df[(result_df['ALGOR'] == algo) & (result_df['STOP_AND_WAIT'] == sw)].copy()
        
        if not filtered_data.empty:
            rounded_values = filtered_data[metric].astype(float).round(2)
            filtered_data['Metric'] = metric
            filtered_data['Simulation Type'] = sim_type
            filtered_data['Value'] = rounded_values
            agg_data = pd.concat([agg_data, filtered_data[['robot_name', 'Metric', 'Simulation Type', 'Value']]], sort=False)
            
sns.set_style("whitegrid")
sns.set_palette("husl")

ordered_robot_names = sorted(agg_data['robot_name'].unique(), key=lambda x: int(x.split('robot')[1]))

fig, axes = plt.subplots(2, 2, figsize=(12, 10))

fatal_collisions_counts = {
    'APF': result_df[(result_df['ALGOR'] == '1') & (result_df['STOP_AND_WAIT'] == '0')]['fatal_collisions'].astype(int).sum(),
    'APF with SAW': result_df[(result_df['ALGOR'] == '1') & (result_df['STOP_AND_WAIT'] == '1')]['fatal_collisions'].astype(int).sum(),
    'ORCA': result_df[(result_df['ALGOR'] == '2') & (result_df['STOP_AND_WAIT'] == '0')]['fatal_collisions'].astype(int).sum(),
    'ORCA with SAW': result_df[(result_df['ALGOR'] == '2') & (result_df['STOP_AND_WAIT'] == '1')]['fatal_collisions'].astype(int).sum()
}

fatal_collisions_subtitle = 'Not Avoided Collisions: ' + \
    'APF ({}) '.format(fatal_collisions_counts['APF']) + \
    'APF with SAW ({}) '.format(fatal_collisions_counts['APF with SAW']) + \
    'ORCA ({})  '.format(fatal_collisions_counts['ORCA']) + \
    'ORCA with SAW ({}) '.format(fatal_collisions_counts['ORCA with SAW'])

fig.suptitle('Comparison of Metrics by Robot and Simulation Type\n{}'.format(fatal_collisions_subtitle), fontsize=13)


for i, metric in enumerate(metrics):
    row = i // 2
    col = i % 2
    ax = axes[row, col]
    
    if metric in ['navigation_time', 'CA_time']:
        ax.axhline(y=optimal_time_mean, color='red', linewidth=2, linestyle='solid', label='Time Limit')

    sns.barplot(data=agg_data[agg_data['Metric'] == metric], x='robot_name', y='Value', hue='Simulation Type', ax=ax,
                order=ordered_robot_names, dodge=True)
    
    if metric == "navigation_time":
        ax.set_title('Navigation Time')
        ax.set_ylabel("Navigation Time")
    elif metric == "CA_time":
        ax.set_title('Collision Avoidance Time')
        ax.set_ylabel("Collision Avoidance Time")
    elif metric == "avoided_collisions":
        ax.set_title('Avoided Collisions')
        ax.set_ylabel("Avoided Collisions")
    else:
        ax.set_title('Distance Traveled')
        ax.set_ylabel("Distance Traveled")
    ax.set_xlabel('Robot Name')
    ax.legend(loc='upper left', prop={'size': 9})
    
    ax.set_yticklabels(['{:,.0f}'.format(x) for x in ax.get_yticks()])

last_ax = axes[-1, -1]
handles, labels = last_ax.get_legend_handles_labels()
last_ax.legend(handles=handles, labels=labels, loc='upper right', prop={'size': 9})

plt.tight_layout(rect=[0, 0.03, 1, 0.95])

plt.savefig('/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/HD_8AUV_1.png')

plt.show()

file_path = '/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/comparative_8robots_2.csv'

df = pd.read_csv(file_path, header=None, encoding = "utf-8")

if '%time' in df.iloc[0].values:
    df = df.drop(columns=df.columns[0])

result_columns = ['robot_name', 'ALGOR', 'STOP_AND_WAIT', 'W1', 'W2', 'time_step', 'time_horizon',
                  'sphere_radius', 'CRIT_DIST', 'DES_VEL', 'CA_ROT_VEL',  'K_ROT_MAX',
                  'K_ROT_MIN', 'ORI_ERROR', 'robot_stuck', 'avoided_collisions',  'fatal_collisions', 
                  'navigation_time', 'CA_time', 'distance_travelled', 'optimal_time', 'optimal_distance_travelled']

result_df = pd.DataFrame(columns=result_columns)

filtered_df = df[df.iloc[:, 0].str.startswith('field.') == False].copy()

filtered_df.reset_index(drop=True, inplace=True)

result_df['robot_name'] = filtered_df.iloc[:, 0]

result_df[result_columns[1:]] = filtered_df.iloc[:, 1:]

print(result_df)

result_df['optimal_time'] = pd.to_numeric(result_df['optimal_time'], errors='coerce')
optimal_time_mean = 10 * result_df['optimal_time'].mean()

metrics = ['navigation_time', 'CA_time', 'avoided_collisions', 'distance_travelled']

simulation_types = {
    ('1', '1'): 'APF with SAW',
    ('1', '0'): 'APF',
    ('2', '1'): 'ORCA with SAW',
    ('2', '0'): 'ORCA'
}

agg_data = pd.DataFrame(columns=['Robot Name', 'Metric', 'Simulation Type', 'Value'])

for metric in metrics:
    for (algo, sw), sim_type in simulation_types.items():
        filtered_data = result_df[(result_df['ALGOR'] == algo) & (result_df['STOP_AND_WAIT'] == sw)].copy()
        
        if not filtered_data.empty:
            rounded_values = filtered_data[metric].astype(float).round(2)
            filtered_data['Metric'] = metric
            filtered_data['Simulation Type'] = sim_type
            filtered_data['Value'] = rounded_values
            agg_data = pd.concat([agg_data, filtered_data[['robot_name', 'Metric', 'Simulation Type', 'Value']]], sort=False)
            
sns.set_style("whitegrid")
sns.set_palette("husl")

ordered_robot_names = sorted(agg_data['robot_name'].unique(), key=lambda x: int(x.split('robot')[1]))

fig, axes = plt.subplots(2, 2, figsize=(12, 10))

fatal_collisions_counts = {
    'APF': result_df[(result_df['ALGOR'] == '1') & (result_df['STOP_AND_WAIT'] == '0')]['fatal_collisions'].astype(int).sum(),
    'APF with SAW': result_df[(result_df['ALGOR'] == '1') & (result_df['STOP_AND_WAIT'] == '1')]['fatal_collisions'].astype(int).sum(),
    'ORCA': result_df[(result_df['ALGOR'] == '2') & (result_df['STOP_AND_WAIT'] == '0')]['fatal_collisions'].astype(int).sum(),
    'ORCA with SAW': result_df[(result_df['ALGOR'] == '2') & (result_df['STOP_AND_WAIT'] == '1')]['fatal_collisions'].astype(int).sum()
}

fatal_collisions_subtitle = 'Not Avoided Collisions: ' + \
    'APF ({}) '.format(fatal_collisions_counts['APF']) + \
    'APF with SAW ({}) '.format(fatal_collisions_counts['APF with SAW']) + \
    'ORCA ({})  '.format(fatal_collisions_counts['ORCA']) + \
    'ORCA with SAW ({}) '.format(fatal_collisions_counts['ORCA with SAW'])

fig.suptitle('Comparison of Metrics by Robot and Simulation Type\n{}'.format(fatal_collisions_subtitle), fontsize=13)


for i, metric in enumerate(metrics):
    row = i // 2
    col = i % 2
    ax = axes[row, col]
    
    if metric in ['navigation_time', 'CA_time']:
        ax.axhline(y=optimal_time_mean, color='red', linewidth=2, linestyle='solid', label='Time Limit')

    sns.barplot(data=agg_data[agg_data['Metric'] == metric], x='robot_name', y='Value', hue='Simulation Type', ax=ax,
                order=ordered_robot_names, dodge=True)
    
    if metric == "navigation_time":
        ax.set_title('Navigation Time')
        ax.set_ylabel("Navigation Time")
    elif metric == "CA_time":
        ax.set_title('Collision Avoidance Time')
        ax.set_ylabel("Collision Avoidance Time")
    elif metric == "avoided_collisions":
        ax.set_title('Avoided Collisions')
        ax.set_ylabel("Avoided Collisions")
    else:
        ax.set_title('Distance Traveled')
        ax.set_ylabel("Distance Traveled")
    ax.set_xlabel('Robot Name')
    ax.legend(loc='upper left', prop={'size': 9})
    
    ax.set_yticklabels(['{:,.0f}'.format(x) for x in ax.get_yticks()])

last_ax = axes[-1, -1]
handles, labels = last_ax.get_legend_handles_labels()
last_ax.legend(handles=handles, labels=labels, loc='upper right', prop={'size': 9})

plt.tight_layout(rect=[0, 0.03, 1, 0.95])

plt.savefig('/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/HD_8AUV_2.png')

plt.show()

file_path = '/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/low_density/comparative_4robots.csv'

df = pd.read_csv(file_path, header=None, encoding = "utf-8")

if '%time' in df.iloc[0].values:
    df = df.drop(columns=df.columns[0])

result_columns = ['robot_name', 'ALGOR', 'STOP_AND_WAIT', 'W1', 'W2', 'time_step', 'time_horizon',
                  'sphere_radius', 'CRIT_DIST', 'DES_VEL', 'CA_ROT_VEL',  'K_ROT_MAX',
                  'K_ROT_MIN', 'ORI_ERROR', 'robot_stuck', 'avoided_collisions',  'fatal_collisions', 
                  'navigation_time', 'CA_time', 'distance_travelled', 'optimal_time', 'optimal_distance_travelled']

result_df = pd.DataFrame(columns=result_columns)

filtered_df = df[df.iloc[:, 0].str.startswith('field.') == False].copy()

filtered_df.reset_index(drop=True, inplace=True)

result_df['robot_name'] = filtered_df.iloc[:, 0]

result_df[result_columns[1:]] = filtered_df.iloc[:, 1:]

print(result_df)

result_df['optimal_time'] = pd.to_numeric(result_df['optimal_time'], errors='coerce')
optimal_time_mean = 10 * result_df['optimal_time'].mean()

metrics = ['navigation_time', 'CA_time', 'avoided_collisions', 'distance_travelled']

simulation_types = {
    ('1', '1'): 'APF with SAW',
    ('1', '0'): 'APF',
    ('2', '1'): 'ORCA with SAW',
    ('2', '0'): 'ORCA'
}

agg_data = pd.DataFrame(columns=['Robot Name', 'Metric', 'Simulation Type', 'Value'])

for metric in metrics:
    for (algo, sw), sim_type in simulation_types.items():
        filtered_data = result_df[(result_df['ALGOR'] == algo) & (result_df['STOP_AND_WAIT'] == sw)].copy()
        
        if not filtered_data.empty:
            rounded_values = filtered_data[metric].astype(float).round(2)
            filtered_data['Metric'] = metric
            filtered_data['Simulation Type'] = sim_type
            filtered_data['Value'] = rounded_values
            agg_data = pd.concat([agg_data, filtered_data[['robot_name', 'Metric', 'Simulation Type', 'Value']]], sort=False)
            
sns.set_style("whitegrid")
sns.set_palette("husl")

ordered_robot_names = sorted(agg_data['robot_name'].unique(), key=lambda x: int(x.split('robot')[1]))

fig, axes = plt.subplots(2, 2, figsize=(12, 10))

fatal_collisions_counts = {
    'APF': result_df[(result_df['ALGOR'] == '1') & (result_df['STOP_AND_WAIT'] == '0')]['fatal_collisions'].astype(int).sum(),
    'APF with SAW': result_df[(result_df['ALGOR'] == '1') & (result_df['STOP_AND_WAIT'] == '1')]['fatal_collisions'].astype(int).sum(),
    'ORCA': result_df[(result_df['ALGOR'] == '2') & (result_df['STOP_AND_WAIT'] == '0')]['fatal_collisions'].astype(int).sum(),
    'ORCA with SAW': result_df[(result_df['ALGOR'] == '2') & (result_df['STOP_AND_WAIT'] == '1')]['fatal_collisions'].astype(int).sum()
}

fatal_collisions_subtitle = 'Not Avoided Collisions: ' + \
    'APF ({}) '.format(fatal_collisions_counts['APF']) + \
    'APF with SAW ({}) '.format(fatal_collisions_counts['APF with SAW']) + \
    'ORCA ({})  '.format(fatal_collisions_counts['ORCA']) + \
    'ORCA with SAW ({}) '.format(fatal_collisions_counts['ORCA with SAW'])

fig.suptitle('Comparison of Metrics by Robot and Simulation Type\n{}'.format(fatal_collisions_subtitle), fontsize=13)


for i, metric in enumerate(metrics):
    row = i // 2
    col = i % 2
    ax = axes[row, col]
    
    if metric in ['navigation_time', 'CA_time']:
        ax.axhline(y=optimal_time_mean, color='red', linewidth=2, linestyle='solid', label='Time Limit')

    sns.barplot(data=agg_data[agg_data['Metric'] == metric], x='robot_name', y='Value', hue='Simulation Type', ax=ax,
                order=ordered_robot_names, dodge=True)
    
    if metric == "navigation_time":
        ax.set_title('Navigation Time')
        ax.set_ylabel("Navigation Time")
    elif metric == "CA_time":
        ax.set_title('Collision Avoidance Time')
        ax.set_ylabel("Collision Avoidance Time")
    elif metric == "avoided_collisions":
        ax.set_title('Avoided Collisions')
        ax.set_ylabel("Avoided Collisions")
    else:
        ax.set_title('Distance Traveled')
        ax.set_ylabel("Distance Traveled")
    ax.set_xlabel('Robot Name')
    ax.legend(loc='upper left', prop={'size': 9})
    
    ax.set_yticklabels(['{:,.0f}'.format(x) for x in ax.get_yticks()])

last_ax = axes[-1, -1]
handles, labels = last_ax.get_legend_handles_labels()
last_ax.legend(handles=handles, labels=labels, loc='upper right', prop={'size': 9})

plt.tight_layout(rect=[0, 0.03, 1, 0.95])

plt.savefig('/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/low_density/LD_4AUV.png')

plt.show()

file_path = '/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/low_density/comparative_8robots.csv'

df = pd.read_csv(file_path, header=None, encoding = "utf-8")

if '%time' in df.iloc[0].values:
    df = df.drop(columns=df.columns[0])

result_columns = ['robot_name', 'ALGOR', 'STOP_AND_WAIT', 'W1', 'W2', 'time_step', 'time_horizon',
                  'sphere_radius', 'CRIT_DIST', 'DES_VEL', 'CA_ROT_VEL',  'K_ROT_MAX',
                  'K_ROT_MIN', 'ORI_ERROR', 'robot_stuck', 'avoided_collisions',  'fatal_collisions', 
                  'navigation_time', 'CA_time', 'distance_travelled', 'optimal_time', 'optimal_distance_travelled']

result_df = pd.DataFrame(columns=result_columns)

filtered_df = df[df.iloc[:, 0].str.startswith('field.') == False].copy()

filtered_df.reset_index(drop=True, inplace=True)

result_df['robot_name'] = filtered_df.iloc[:, 0]

result_df[result_columns[1:]] = filtered_df.iloc[:, 1:]

print(result_df)

result_df['optimal_time'] = pd.to_numeric(result_df['optimal_time'], errors='coerce')
optimal_time_mean = 10 * result_df['optimal_time'].mean()

metrics = ['navigation_time', 'CA_time', 'avoided_collisions', 'distance_travelled']

simulation_types = {
    ('1', '1'): 'APF with SAW',
    ('1', '0'): 'APF',
    ('2', '1'): 'ORCA with SAW',
    ('2', '0'): 'ORCA'
}

agg_data = pd.DataFrame(columns=['Robot Name', 'Metric', 'Simulation Type', 'Value'])

for metric in metrics:
    for (algo, sw), sim_type in simulation_types.items():
        filtered_data = result_df[(result_df['ALGOR'] == algo) & (result_df['STOP_AND_WAIT'] == sw)].copy()
        
        if not filtered_data.empty:
            rounded_values = filtered_data[metric].astype(float).round(2)
            filtered_data['Metric'] = metric
            filtered_data['Simulation Type'] = sim_type
            filtered_data['Value'] = rounded_values
            agg_data = pd.concat([agg_data, filtered_data[['robot_name', 'Metric', 'Simulation Type', 'Value']]], sort=False)
            
sns.set_style("whitegrid")
sns.set_palette("husl")

ordered_robot_names = sorted(agg_data['robot_name'].unique(), key=lambda x: int(x.split('robot')[1]))

fig, axes = plt.subplots(2, 2, figsize=(12, 10))

fatal_collisions_counts = {
    'APF': result_df[(result_df['ALGOR'] == '1') & (result_df['STOP_AND_WAIT'] == '0')]['fatal_collisions'].astype(int).sum(),
    'APF with SAW': result_df[(result_df['ALGOR'] == '1') & (result_df['STOP_AND_WAIT'] == '1')]['fatal_collisions'].astype(int).sum(),
    'ORCA': result_df[(result_df['ALGOR'] == '2') & (result_df['STOP_AND_WAIT'] == '0')]['fatal_collisions'].astype(int).sum(),
    'ORCA with SAW': result_df[(result_df['ALGOR'] == '2') & (result_df['STOP_AND_WAIT'] == '1')]['fatal_collisions'].astype(int).sum()
}

fatal_collisions_subtitle = 'Not Avoided Collisions: ' + \
    'APF ({}) '.format(fatal_collisions_counts['APF']) + \
    'APF with SAW ({}) '.format(fatal_collisions_counts['APF with SAW']) + \
    'ORCA ({})  '.format(fatal_collisions_counts['ORCA']) + \
    'ORCA with SAW ({}) '.format(fatal_collisions_counts['ORCA with SAW'])

fig.suptitle('Comparison of Metrics by Robot and Simulation Type\n{}'.format(fatal_collisions_subtitle), fontsize=13)


for i, metric in enumerate(metrics):
    row = i // 2
    col = i % 2
    ax = axes[row, col]
    
    if metric in ['navigation_time', 'CA_time']:
        ax.axhline(y=optimal_time_mean, color='red', linewidth=2, linestyle='solid', label='Time Limit')

    sns.barplot(data=agg_data[agg_data['Metric'] == metric], x='robot_name', y='Value', hue='Simulation Type', ax=ax,
                order=ordered_robot_names, dodge=True)
    
    if metric == "navigation_time":
        ax.set_title('Navigation Time')
        ax.set_ylabel("Navigation Time")
    elif metric == "CA_time":
        ax.set_title('Collision Avoidance Time')
        ax.set_ylabel("Collision Avoidance Time")
    elif metric == "avoided_collisions":
        ax.set_title('Avoided Collisions')
        ax.set_ylabel("Avoided Collisions")
    else:
        ax.set_title('Distance Traveled')
        ax.set_ylabel("Distance Traveled")
    ax.set_xlabel('Robot Name')
    ax.legend(loc='upper left', prop={'size': 9})
    
    ax.set_yticklabels(['{:,.0f}'.format(x) for x in ax.get_yticks()])

last_ax = axes[-1, -1]
handles, labels = last_ax.get_legend_handles_labels()
last_ax.legend(handles=handles, labels=labels, loc='upper right', prop={'size': 9})

plt.tight_layout(rect=[0, 0.03, 1, 0.95])

plt.savefig('/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/low_density/LD_8AUV_1.png')

plt.show()

file_path = '/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/low_density/comparative_8robots_2.csv'

df = pd.read_csv(file_path, header=None, encoding = "utf-8")

if '%time' in df.iloc[0].values:
    df = df.drop(columns=df.columns[0])

result_columns = ['robot_name', 'ALGOR', 'STOP_AND_WAIT', 'W1', 'W2', 'time_step', 'time_horizon',
                  'sphere_radius', 'CRIT_DIST', 'DES_VEL', 'CA_ROT_VEL',  'K_ROT_MAX',
                  'K_ROT_MIN', 'ORI_ERROR', 'robot_stuck', 'avoided_collisions',  'fatal_collisions', 
                  'navigation_time', 'CA_time', 'distance_travelled', 'optimal_time', 'optimal_distance_travelled']

result_df = pd.DataFrame(columns=result_columns)

filtered_df = df[df.iloc[:, 0].str.startswith('field.') == False].copy()

filtered_df.reset_index(drop=True, inplace=True)

result_df['robot_name'] = filtered_df.iloc[:, 0]

result_df[result_columns[1:]] = filtered_df.iloc[:, 1:]

print(result_df)

result_df['optimal_time'] = pd.to_numeric(result_df['optimal_time'], errors='coerce')
optimal_time_mean = 10 * result_df['optimal_time'].mean()

metrics = ['navigation_time', 'CA_time', 'avoided_collisions', 'distance_travelled']

simulation_types = {
    ('1', '1'): 'APF with SAW',
    ('1', '0'): 'APF',
    ('2', '1'): 'ORCA with SAW',
    ('2', '0'): 'ORCA'
}

agg_data = pd.DataFrame(columns=['Robot Name', 'Metric', 'Simulation Type', 'Value'])

for metric in metrics:
    for (algo, sw), sim_type in simulation_types.items():
        filtered_data = result_df[(result_df['ALGOR'] == algo) & (result_df['STOP_AND_WAIT'] == sw)].copy()
        
        if not filtered_data.empty:
            rounded_values = filtered_data[metric].astype(float).round(2)
            filtered_data['Metric'] = metric
            filtered_data['Simulation Type'] = sim_type
            filtered_data['Value'] = rounded_values
            agg_data = pd.concat([agg_data, filtered_data[['robot_name', 'Metric', 'Simulation Type', 'Value']]], sort=False)
            
sns.set_style("whitegrid")
sns.set_palette("husl")

ordered_robot_names = sorted(agg_data['robot_name'].unique(), key=lambda x: int(x.split('robot')[1]))

fig, axes = plt.subplots(2, 2, figsize=(12, 10))

fatal_collisions_counts = {
    'APF': result_df[(result_df['ALGOR'] == '1') & (result_df['STOP_AND_WAIT'] == '0')]['fatal_collisions'].astype(int).sum(),
    'APF with SAW': result_df[(result_df['ALGOR'] == '1') & (result_df['STOP_AND_WAIT'] == '1')]['fatal_collisions'].astype(int).sum(),
    'ORCA': result_df[(result_df['ALGOR'] == '2') & (result_df['STOP_AND_WAIT'] == '0')]['fatal_collisions'].astype(int).sum(),
    'ORCA with SAW': result_df[(result_df['ALGOR'] == '2') & (result_df['STOP_AND_WAIT'] == '1')]['fatal_collisions'].astype(int).sum()
}

fatal_collisions_subtitle = 'Not Avoided Collisions: ' + \
    'APF ({}) '.format(fatal_collisions_counts['APF']) + \
    'APF with SAW ({}) '.format(fatal_collisions_counts['APF with SAW']) + \
    'ORCA ({})  '.format(fatal_collisions_counts['ORCA']) + \
    'ORCA with SAW ({}) '.format(fatal_collisions_counts['ORCA with SAW'])

fig.suptitle('Comparison of Metrics by Robot and Simulation Type\n{}'.format(fatal_collisions_subtitle), fontsize=13)


for i, metric in enumerate(metrics):
    row = i // 2
    col = i % 2
    ax = axes[row, col]
    
    if metric in ['navigation_time', 'CA_time']:
        ax.axhline(y=optimal_time_mean, color='red', linewidth=2, linestyle='solid', label='Time Limit')

    sns.barplot(data=agg_data[agg_data['Metric'] == metric], x='robot_name', y='Value', hue='Simulation Type', ax=ax,
                order=ordered_robot_names, dodge=True)
    
    if metric == "navigation_time":
        ax.set_title('Navigation Time')
        ax.set_ylabel("Navigation Time")
    elif metric == "CA_time":
        ax.set_title('Collision Avoidance Time')
        ax.set_ylabel("Collision Avoidance Time")
    elif metric == "avoided_collisions":
        ax.set_title('Avoided Collisions')
        ax.set_ylabel("Avoided Collisions")
    else:
        ax.set_title('Distance Traveled')
        ax.set_ylabel("Distance Traveled")
    ax.set_xlabel('Robot Name')
    ax.legend(loc='upper left', prop={'size': 9})
    
    ax.set_yticklabels(['{:,.0f}'.format(x) for x in ax.get_yticks()])

last_ax = axes[-1, -1]
handles, labels = last_ax.get_legend_handles_labels()
last_ax.legend(handles=handles, labels=labels, loc='upper right', prop={'size': 9})

plt.tight_layout(rect=[0, 0.03, 1, 0.95])

plt.savefig('/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/low_density/LD_8AUV_2.png')

plt.show()
