import pandas as pd

def delete_columns(input_csv, output_csv, columns_to_delete):
    # CSV ファイルを読み込む
    df = pd.read_csv(input_csv)

    # 指定した列を削除する
    for column in columns_to_delete:
        if column in df.columns:
            df = df.drop(columns=[column])
            print(f"Column '{column}' deleted successfully.")
        else:
            print(f"Column '{column}' not found.")

    # 新しい CSV ファイルを生成する
    df.to_csv(output_csv, index=False)
    print(f"New CSV file '{output_csv}' created successfully.")

# 使用例
input_file = '/home/kajilab/catkin_ws/src/rosbag_to_csv/scripts/test.csv'  # 入力CSVファイル名
output_file = '/home/kajilab/catkin_ws/src/rosbag_to_csv/scripts/test_remake.csv'  # 出力CSVファイル名
columns_to_remove = ['.layout.dim', '.layout.data_offset']  # 削除する列の名前

delete_columns(input_file, output_file, columns_to_remove)

