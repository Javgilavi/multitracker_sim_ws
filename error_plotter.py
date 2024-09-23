import matplotlib.pyplot as plt

# Parse error data by ID from the file
data_by_id = {}
with open('error_data.txt', 'r') as file:
    for line in file:
        parts = line.split()
        id = int(parts[1])
        pos_error_x, pos_error_y, pos_error_z = map(float, parts[2:5])
        ori_error_x, ori_error_y, ori_error_z, ori_error_w = map(float, parts[5:])

        if id not in data_by_id:
            data_by_id[id] = {'pos_x': [], 'pos_y': [], 'pos_z': [],
                              'ori_x': [], 'ori_y': [], 'ori_z': [], 'ori_w': []}
        
        data_by_id[id]['pos_x'].append(pos_error_x)
        data_by_id[id]['pos_y'].append(pos_error_y)
        data_by_id[id]['pos_z'].append(pos_error_z)
        data_by_id[id]['ori_x'].append(ori_error_x)
        data_by_id[id]['ori_y'].append(ori_error_y)
        data_by_id[id]['ori_z'].append(ori_error_z)
        data_by_id[id]['ori_w'].append(ori_error_w)

# Plot for each ID
for id, data in data_by_id.items():
    plt.figure()
    plt.plot(data['pos_x'], label='X Error')
    plt.plot(data['pos_y'], label='Y Error')
    plt.plot(data['pos_z'], label='Z Error')
    plt.title(f'Position Error for ID {id}')
    plt.legend()
    plt.show()

    plt.figure()
    plt.plot(data['ori_x'], label='Orientation X Error')
    plt.plot(data['ori_y'], label='Orientation Y Error')
    plt.plot(data['ori_z'], label='Orientation Z Error')
    plt.plot(data['ori_w'], label='Orientation W Error')
    plt.title(f'Orientation Error for ID {id}')
    plt.legend()
    plt.show()
