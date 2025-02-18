import csv

def csv_to_txt(csv_filename, txt_filename):
    with open(csv_filename, 'r', newline='', encoding='utf-8') as csv_file:
        with open(txt_filename, 'w', encoding='utf-8') as txt_file:
            reader = csv.reader(csv_file)
            for row in reader:
                txt_file.write(' '.join(row) + '\n')

# Ejemplo de uso
csv_to_txt('/home/luis/datasets/SynColon/juanjo/seq0/trajectory.csv', '/home/luis/datasets/SynColon/juanjo/seq0/trajectory.txt')