# -*- coding: utf-8 -*-
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

file_path = '/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/testing_results.csv'

# Cargar el archivo CSV en un DataFrame sin encabezados
df = pd.read_csv(file_path, header=None, encoding = "utf-8")

# Eliminar la columna '%time' si es necesario
if '%time' in df.iloc[0].values:
    df = df.drop(columns=df.columns[0])  # Eliminar la primera columna que contiene '%time'

if 'field.robot_name' in df.iloc[0].values:
    df = df.drop(columns=df.columns[0])

print(df)

df.columns = range(len(df.columns))

result_columns = ['ALGOR', 'STOP_AND_WAIT', 'CRIT_DIST', 'DES_VEL', 'AZIMUTH_ERROR', 'W1', 'W2',
                  'ROT_VEL', 'K_ROT_MAX', 'K_ROT_MIN', 'time_step','time_horizon', 'sphere_radius',
                  'robot_stuck', 'avoided_collisions', 'fatal_collisions',
                  'navigation_time', 'CA_time', 'distance_travelled', 'optimal_time', 'optimal_distance_travelled']

result_df = pd.DataFrame(columns=result_columns)

current_group = None  
group_data = []       

for index, row in df.iterrows():
    if row.iloc[0] == 'field.ALGOR':
        # Se encontró una fila de campos, procesar el grupo anterior (si existe)
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
            current_group[result_columns.index('AZIMUTH_ERROR')] = current_AE
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
current_group[result_columns.index('AZIMUTH_ERROR')] = current_AE
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

result_df = result_df.reset_index(drop=True)

PF_df = result_df[result_df['ALGOR'] == '1']
RVO_df = result_df[result_df['ALGOR'] == '2']

PF_initial_num_sim = len(PF_df)
RVO_initial_num_sim = len(RVO_df)

PF_df = PF_df[(PF_df["fatal_collisions"] == 0) & (PF_df["robot_stuck"] == 0)]
RVO_df = RVO_df[(RVO_df["fatal_collisions"] == 0) & (RVO_df["robot_stuck"] == 0)]

PF_df = PF_df.reset_index(drop=True)
RVO_df = RVO_df.reset_index(drop=True)

PF_num_sim = len(PF_df)
RVO_num_sim = len(RVO_df)

print(PF_df)
print(RVO_df)

group_df = [PF_df, RVO_df]
for current_df in group_df:
    if PF_df.equals(current_df):
        plt.figure(figsize=(12, 12), num='Potential Fields simulations ({} successful simulations out of {})'.format(PF_num_sim, PF_initial_num_sim))

    else:
        plt.figure(figsize=(12, 12), num='RVO2-3D simulations ({} successful simulations out of {})'.format(RVO_num_sim, RVO_initial_num_sim))

    plot_variables = [
        ("CRIT_DIST", "avoided_collisions", "avoided_collisions vs CRIT_DIST"),
        ("CRIT_DIST", "navigation_time", "navigation_time vs CRIT_DIST"),
        ("CRIT_DIST", "CA_time", "CA_time vs CRIT_DIST"),
        ("CRIT_DIST", "distance_travelled", "distance_travelled vs CRIT_DIST"),
        ("AZIMUTH_ERROR", "avoided_collisions", "avoided_collisions vs AZIMUTH_ERROR"),
        ("AZIMUTH_ERROR", "navigation_time", "navigation_time vs AZIMUTH_ERROR"),
        ("AZIMUTH_ERROR", "CA_time", "CA_time vs AZIMUTH_ERROR"),
        ("AZIMUTH_ERROR", "distance_travelled", "distance_travelled vs AZIMUTH_ERROR"),
        ("W2", "avoided_collisions", "avoided_collisions vs DES_VEL"),
        ("W2", "navigation_time", "navigation_time vs DES_VEL"),
        ("W2", "CA_time", "CA_time vs DES_VEL"),
        ("W2", "distance_travelled", "distance_travelled vs DES_VEL"),
    ]

    for i, (x_var, y_var, title) in enumerate(plot_variables, start=1):
        plt.subplot(3, 4, i)
        sns.boxplot(x=x_var, y=y_var, data=current_df)
        plt.xlabel(x_var)
        plt.ylabel(y_var)
        plt.title(title)

    plt.tight_layout()
    plt.show()

PF_df = PF_df.sort_values(by=["navigation_time", "distance_travelled", "CA_time", "avoided_collisions"], ascending=[True, True, True, True])
PF_best_result = PF_df.iloc[0]
print("")
print("Potential Fields best simulation")
print("")
print(PF_best_result)

RVO_df = RVO_df.sort_values(by=["navigation_time", "distance_travelled", "CA_time", "avoided_collisions"], ascending=[True, True, True, True])
RVO_best_result = RVO_df.iloc[0]
print("")
print("RVO2-3D best simulation")
print("")    
print(RVO_best_result)

file_path = '/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/comparative_4robots.csv'

df = pd.read_csv(file_path, header=None, encoding = "utf-8")

if '%time' in df.iloc[0].values:
    df = df.drop(columns=df.columns[0])

result_columns = ['robot_name', 'ALGOR', 'STOP_AND_WAIT', 'W1', 'W2', 'time_step', 'time_horizon',
                  'sphere_radius', 'CRIT_DIST', 'DES_VEL', 'CA_ROT_VEL',  'K_ROT_MAX',
                  'K_ROT_MIN', 'AZIMUTH_ERROR', 'robot_stuck', 'avoided_collisions',  'fatal_collisions', 
                  'navigation_time', 'CA_time', 'distance_travelled', 'optimal_time', 'optimal_distance_travelled']

result_df = pd.DataFrame(columns=result_columns)

filtered_df = df[df.iloc[:, 0].str.startswith('field.') == False].copy()

filtered_df.reset_index(drop=True, inplace=True)

result_df['robot_name'] = filtered_df.iloc[:, 0]

result_df[result_columns[1:]] = filtered_df.iloc[:, 1:]

print(result_df)


metrics = ['navigation_time', 'CA_time', 'avoided_collisions', 'distance_travelled']

simulation_types = {
    ('1', '1'): 'Potential Fields with SAW',
    ('1', '0'): 'Potential Fields',
    ('2', '1'): 'RVO2-3D with SAW',
    ('2', '0'): 'RVO2-3D'
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
fig.suptitle('Comparison of Metrics by Robot and Simulation Type', fontsize=16)

for i, metric in enumerate(metrics):
    row = i // 2
    col = i % 2
    ax = axes[row, col]
    
    sns.barplot(data=agg_data[agg_data['Metric'] == metric], x='robot_name', y='Value', hue='Simulation Type', ax=ax,
                order=ordered_robot_names, dodge=True)
    
    ax.set_title('{}'.format(metric.capitalize()))
    ax.set_xlabel('Robot Name')
    ax.set_ylabel(metric)
    ax.legend(title='Simulation Type', loc='upper left', prop={'size': 7})
    
    ax.set_yticklabels(['{:,.0f}'.format(x) for x in ax.get_yticks()])

last_ax = axes[-1, -1]
handles, labels = last_ax.get_legend_handles_labels()
last_ax.legend(handles=handles, labels=labels, title='Simulation Type', loc='upper left', prop={'size': 7})

plt.tight_layout(rect=[0, 0.03, 1, 0.95])

plt.show()

