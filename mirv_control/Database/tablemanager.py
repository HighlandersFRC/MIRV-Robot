#!/usr/bin/env python3
import sqlite3
from sqlite3 import Error
import time
from sensor_msgs.msg import NavSatFix

"""
Usage Example:

    from tablemanager import TableManager

    my_table_command = '''CREATE TABLE IF NOT EXISTS mytable (
        timestamp double PRIMARY KEY,
        val_1 text NOT NULL,
        val_2 intager NOT NULL
    )'''
    my_table_columns = (
        'val_1',
        'val_2'
    )

    tm = TableManager()
    tm.connect(r'my_table.db')

    tm.create_table(my_table_cmd)
    tm.append_row('my_table', my_table_columns, ('example input', 12345))
    entry = tm.get_last_row('my_table')
    print(entry)

    tm.close()
"""

class TableManager:
    def __init__(self):
        self.conn = None

    def connect(self, db_file):
        #Create a connection to the desired database
        try:
            self.conn = sqlite3.connect(db_file)
        except Error as e:
            print(e)
            self.conn = None

    def create_table(self, table_cmd):
        if self.is_connected():
            self.execute(table_cmd)

    def close(self):
        self.conn.close()
        self.conn = None

    def append_row(self, table_name, table_columns: tuple, data: tuple):
        #Append a row of data to a database
        cmd = f"""INSERT INTO {table_name} {"".join([c for c in str(table_columns) if c != "'" and c != '"'])}
        VALUES {"".join([c for c in str(tuple("?" for i in range(len(table_columns)))) if c != "'" and c != '"'])}
        """
        self.execute(cmd, data)
        self.conn.commit()

    def deployed_pilit(self, gps_pos: NavSatFix, side: str, table_columns: tuple):
        prev_entry = self.get_last_row("pilits")
        if not prev_entry: 
            return
        prev_entry = prev_entry[:-1]
        prev_times = prev_entry[3::5]
        print(prev_entry)
        print(prev_times)
        index = prev_times.index(min(prev_times))
        index *= 5
        index += 3
        new_entry = prev_entry
        new_entry[0] = time.time()
        if side == "right":
            if prev_entry[1] > 0:
                new_entry[1] = prev_entry[1] - 1
            else: 
                print("Right conveyor is already empty")
                return
        elif side == "left":
            if prev_entry[2] > 0:
                new_entry[2] = prev_entry[2] - 1
            else: 
                print("Left conveyor is already empty")
                return
        new_entry[index] = time.time()
        new_entry[index + 1] = "deployed"
        new_entry[index + 2] = gps_pos.latitude
        new_entry[index + 3] = gps_pos.longitude
        new_entry[index + 4] = gps_pos.altitude
        self.append_row("pilits", table_columns, tuple(new_entry))

    def retrieved_pilit(self, gps_pos: NavSatFix, side: str, table_columns: tuple):
        prev_entry = self.get_last_row("pilits")
        if not prev_entry: 
            return
        prev_entry = prev_entry[:-1]
        prev_locations = [prev_entry[5::5], prev_entry[6::5], prev_entry[4::5]]
        distances = []
        for i in range(len(prev_locations[0])):
            if prev_locations[2][i] == "deployed":
                distances.append(abs(gps_pos.latitude - prev_locations[0][i]) + abs(gps_pos.longitude - prev_locations[1][i]))
            else:
                distances.append(9999999)
        print(distances)
        if not distances:
            print("All pilits are already stored")
            return
        index = distances.index(min(distances))
        new_entry = prev_entry
        new_entry[0] = time.time()
        if side == "right":
            new_entry[1] = prev_entry[1] + 1
        elif side == "left":
            new_entry[2] = prev_entry[2] + 1
        new_entry[index * 5 + 3] = time.time()
        new_entry[index * 5 + 4] = "stored"
        new_entry[index * 5 + 5] = 0
        new_entry[index * 5 + 6] = 0
        new_entry[index * 5 + 7] = 0
        print(new_entry)
        self.append_row("pilits", table_columns, tuple(new_entry))

    def get_rows(self, table_name):
        #Returns a 2D list representing the specified table
        try:
            c = self.conn.cursor()
            c.execute(f"SELECT * FROM {table_name}")
            return c.fetchall()
        except Error as e:
            print(e)
            return []

    def get_last_row(self, table_name):
        #Returns a list representing the last (most recent) row in the specified table
        try:
            c = self.conn.cursor()
            c.execute(f"SELECT *, max(timestamp) FROM {table_name}")
            row = c.fetchall()
            return list(row[0])
        except Error as e:
            print(e)
            return []

    def execute(self, cmd, data = None):
        try:
            c = self.conn.cursor()
            if data:
                c.execute(cmd, data)
            else:
                c.execute(cmd)
            c.close()
        except Error as e:
            print(e)

    def is_connected(self):
        return self.conn != None