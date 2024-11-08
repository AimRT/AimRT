# Copyright (c) 2024 The AimRT Authors.
# AimRT is licensed under Mulan PSL v2.

from aimrt_cli.trans import TransBase
import os
import sqlite3
import shutil
from pathlib import Path
import yaml
from dataclasses import dataclass


class IndentDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(IndentDumper, self).increase_indent(flow, False)


class DatabaseManager:
    def __init__(self, db_path: str):
        self.db_path = db_path
        self.conn = None
        self.cursor = None

    def connect(self):
        self.conn = sqlite3.connect(self.db_path)
        self.cursor = self.conn.cursor()
        return self.conn, self.cursor

    def create_tables(self):
        try:
            # create messages table
            self.cursor.execute("""
            CREATE TABLE messages(
                id          INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
                topic_id    INTEGER NOT NULL,
                timestamp   INTEGER NOT NULL,
                data        BLOB NOT NULL)
            """)

            # create topics table
            self.cursor.execute("""
            CREATE TABLE topics(
                id          INTEGER PRIMARY KEY,
                name        TEXT NOT NULL,
                type        TEXT NOT NULL,
                serialization_format   TEXT NOT NULL,
                offered_qos_profiles   TEXT NOT NULL)
            """)

            # create schema table
            self.cursor.execute("""
            CREATE TABLE "schema" (
                "schema_version"	INTEGER,
                "ros_distro"	TEXT NOT NULL,
                PRIMARY KEY("schema_version")
            );
            """)
            self.cursor.execute("""
            INSERT INTO schema (schema_version, ros_distro)
            VALUES (?, ?)
            """, (3, "humble"))

            # create metadata table
            self.cursor.execute("""
            CREATE TABLE metadata(id INTEGER PRIMARY KEY,metadata_version INTEGER NOT NULL,metadata TEXT NOT NULL)
            """)
            self.conn.commit()
        except sqlite3.Error as e:
            self.conn.rollback()
            raise e

    def close(self):
        if self.cursor:
            self.cursor.close()
        if self.conn:
            self.conn.close()


class SingleDbProcess:
    def __init__(self, topic_info_dict: dict, db_path: Path):
        self.message_count = 0
        self.topic_with_message_count = {}
        self.topic_info_dict = topic_info_dict
        self.starting_time_nanoseconds = 100000000000000000000
        self.end_time_nanoseconds = 0
        self.db_path = db_path
        self.get_info()

    def get_bag_info(self, conn, cursor):
        try:
            cursor.execute("SELECT topic_id, timestamp FROM messages")
            rows = sorted(cursor.fetchall())
            if rows:
                self.starting_time_nanoseconds = rows[0][1]
                self.end_time_nanoseconds = rows[-1][1]
                self.message_count = len(rows)
                for row in rows:
                    topic_name = self.topic_info_dict[row[0]].topic_name
                    self.topic_with_message_count[topic_name] = \
                        self.topic_with_message_count.get(topic_name, 0) + 1
        except Exception as e:
            print(f"Error getting single bag info: {e}")
            conn.rollback()

    def get_info(self):
        db_manager = DatabaseManager(str(self.db_path))
        conn, cursor = db_manager.connect()
        try:
            self.get_bag_info(conn, cursor)
        finally:
            db_manager.close()


@dataclass
class TopicInfo:
    topic_id: int
    topic_name: str
    msg_type: str
    serialization_type: str
    message_count: int


class SingleBagTrans(TransBase):
    def __init__(self, input_dir: str, output_dir: str, conn: sqlite3.Connection, cursor: sqlite3.Cursor, id: int):
        super().__init__(output_dir)
        self.input_dir_ = input_dir
        self.topics_list = {}
        self.topic_info_dict = {}
        self.files_list = {}
        self.message_count = 0
        self.starting_time_nanoseconds = 100000000000000000000
        self.end_time_nanoseconds = 0
        self.id = id                # target db message id
        self.conn = conn            # target db connection
        self.cursor = cursor        # target db cursor

    def parse_yaml(self):
        with open(os.path.join(self.input_dir_, "metadata.yaml"), "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        if data is None or data["aimrt_bagfile_information"] is None or data["aimrt_bagfile_information"]["topics"] is None:
            raise Exception("No aimrt_bagfile_information found in metadata.yaml")
        self.topics_list = data["aimrt_bagfile_information"]["topics"]
        for topic in self.topics_list:
            self.topic_info_dict[topic["id"]] = TopicInfo(
                topic["id"], topic["topic_name"], topic["msg_type"], topic["serialization_type"], 0)

        if data["aimrt_bagfile_information"]["files"] is not None:
            self.files_list = data["aimrt_bagfile_information"]["files"]
        else:
            raise Exception("No db files found in metadata.yaml")

    def trans_single_db(self, source_path: Path, topic_map: dict):
        single_bag_info = SingleDbProcess(self.topic_info_dict, source_path)
        self.message_count += single_bag_info.message_count
        self.starting_time_nanoseconds = min(self.starting_time_nanoseconds, single_bag_info.starting_time_nanoseconds)
        self.end_time_nanoseconds = max(self.end_time_nanoseconds, single_bag_info.end_time_nanoseconds)

        conn = sqlite3.connect(source_path)
        print(f"    processing db file: {source_path}")
        cursor = conn.cursor()

        try:
            select_sql = "SELECT id,topic_id, timestamp, data FROM messages"
            cursor.execute(select_sql)
            rows = cursor.fetchall()
            self.cursor.executemany("""
            INSERT INTO messages (id, topic_id, timestamp, data)
            VALUES (?, ?, ?, ?)
            """, [(self.id + row[0], topic_map[self.topic_info_dict[row[1]].topic_name].topic_id, row[2], row[3]) for row in rows])
            for row in rows:
                topic_map[self.topic_info_dict[row[1]].topic_name].message_count += 1
            self.conn.commit()
            print(f"    size of data inserted: {len(rows)} done")
        except Exception as e:
            print(f"    Error updating messages table: {e}")
            self.conn.rollback()
        self.id += len(rows)

    def trans_single_bag(self, topic_map: dict):
        self.parse_yaml()
        print(f"there are {len(self.files_list)} db files in {self.input_dir_}")
        for db_path in self.files_list:
            trans_path = Path(self.output_dir_) / db_path['path']
            self.trans_single_db(Path(self.input_dir_) / db_path['path'], topic_map)
            print(f"    trans_path: {trans_path} done")
        print(f"all db files in {self.input_dir_} done\n")


class AimrtbagToRos2:
    def __init__(self, input_dir: list, output_dir: str):
        self.input_dir_ = input_dir
        self.output_dir_ = output_dir
        self.topic_map = {}
        self.id = 0
        self.message_count = 0
        self.starting_time_nanoseconds = 100000000000000000000
        self.end_time_nanoseconds = 0
        self.topics_list = []
        self.db_manager = None
        self.conn = None
        self.cursor = None

    def create_output_dir(self):
        if os.path.exists(self.output_dir_):
            shutil.rmtree(self.output_dir_)
        os.makedirs(self.output_dir_)
        
        # initialize database
        db_path = os.path.join(self.output_dir_, "rosbag.db3")
        self.db_manager = DatabaseManager(db_path)
        self.conn, self.cursor = self.db_manager.connect()
        self.db_manager.create_tables()

    def parse_yaml(self, input_dir: str):
        with open(os.path.join(input_dir, "metadata.yaml"), "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        if data["aimrt_bagfile_information"] is None or data["aimrt_bagfile_information"]["topics"] is None:
            raise Exception("No topics information found in metadata.yaml")

        topics_list = data["aimrt_bagfile_information"]["topics"]

        for topic in topics_list:
            if topic["topic_name"] not in self.topic_map:
                self.id += 1
                self.topic_map[topic["topic_name"]] = TopicInfo(
                    self.id, topic["topic_name"], topic["msg_type"], topic["serialization_type"], 0)
            else:
                print(f"warning: topic {topic['topic_name']} already exists")

    def format_qos_profiles(self):
        qos_dict = {
            'history': 3,
            'depth': 0,
            'reliability': 1,
            'durability': 2,
            'deadline': {'nsec': 854775807, 'sec': 9223372036},
            'lifespan': {'nsec': 854775807, 'sec': 9223372036},
            'liveliness': 1,
            'liveliness_lease_duration': {'nsec': 854775807, 'sec': 9223372036},
            'avoid_ros_namespace_conventions': False,
        }
        qos_string = '- history: {history}\\n  depth: {depth}\\n  reliability: {reliability}\\n  durability: {durability}\\n  deadline:\\n    sec: {deadline[sec]}\\n    nsec: {deadline[nsec]}\\n  lifespan:\\n    sec: {lifespan[sec]}\\n    nsec: {lifespan[nsec]}\\n  liveliness: {liveliness}\\n  liveliness_lease_duration:\\n    sec: {liveliness_lease_duration[sec]}\\n    nsec: {liveliness_lease_duration[nsec]}\\n  avoid_ros_namespace_conventions: {avoid_ros_namespace_conventions}'.format(
            **qos_dict).replace(
            'True',
            'true').replace(
            'False',
            'false')

        
        return qos_string

    def insert_topics_table(self):
        try:
            qos_dict = [{
                'history': 3,
                'depth': 0,
                'reliability': 1,
                'durability': 2,
                'deadline': {
                    'sec': 9223372036,
                    'nsec': 854775807
                },
                'lifespan': {
                    'sec': 9223372036,
                    'nsec': 854775807
                },
                'liveliness': 1,
                'liveliness_lease_duration': {
                    'sec': 9223372036,
                    'nsec': 854775807
                },
                'avoid_ros_namespace_conventions': False
            }]
            qos_json = yaml.dump(qos_dict, Dumper=IndentDumper, sort_keys=False)

            # Populate the topics table from self.topics_list
            for topic in self.topic_map.values():
                self.cursor.execute("""
                INSERT INTO topics (id, name, type, serialization_format, offered_qos_profiles)
                VALUES (?, ?, ?, ?, ?)
                """, (
                    topic.topic_id,
                    topic.topic_name,
                    topic.msg_type.replace('ros2:', ''),
                    'cdr',  # Use 'cdr' as the default serialization format
                    qos_json
                ))
            self.conn.commit()
        except Exception as e:
            print(f"Error create topics table or insert topics table data, error: {e}")
            self.conn.rollback()

    def update_rosbag_yaml_data(self):
        self.rosbag_yaml_data = {
            "version": 5,
            "storage_identifier": "sqlite3",
            "duration": {
                "nanoseconds": self.end_time_nanoseconds - self.starting_time_nanoseconds
            },
            "starting_time": {
                "nanoseconds_since_epoch": self.starting_time_nanoseconds
            },
            "message_count": self.message_count,
            "topics_with_message_count": [],
            "compression_format": "",
            "compression_mode": "",
            "relative_file_paths": [],
            "files": []
        }

        for topic in self.topic_map.values():
            topic_entry = {
                "topic_metadata": {
                    "name": topic.topic_name,
                    "type": topic.msg_type.replace('ros2:', ''),
                    "serialization_format": "cdr",
                    "offered_qos_profiles": self.format_qos_profiles(),
                },
                "message_count": topic.message_count
            }
            self.rosbag_yaml_data["topics_with_message_count"].append(topic_entry)

        file_entry = {
            "path": "rosbag.db3",
            "starting_time": {
                "nanoseconds_since_epoch": self.starting_time_nanoseconds
            },
            "duration": {
                "nanoseconds": self.end_time_nanoseconds - self.starting_time_nanoseconds
            },
            "message_count": self.message_count
        }
        self.rosbag_yaml_data["relative_file_paths"].append("rosbag.db3")
        self.rosbag_yaml_data["files"].append(file_entry)
        final_yaml_data = {
            "rosbag2_bagfile_information": self.rosbag_yaml_data
        }
        with open(os.path.join(self.output_dir_, "metadata.yaml"), "w") as f:
            yaml_str = yaml.dump(
                final_yaml_data,
                Dumper=IndentDumper,
                default_flow_style=False,
                sort_keys=False,
                indent=2,
                width=1000000)
            yaml_str = yaml_str.replace("\'", "\"")
            f.write(yaml_str)

    def sort_db_data(self):
        print("start sorting messages table by timestamp")
        try:
            self.cursor.execute("""
            CREATE TABLE messages_temp(
                id          INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
                topic_id    INTEGER NOT NULL,
                timestamp   INTEGER NOT NULL,
                data        BLOB NOT NULL)
            """)

            self.cursor.execute("""
            INSERT INTO messages_temp (topic_id, timestamp, data)
            SELECT topic_id, timestamp, data
            FROM messages
            ORDER BY timestamp ASC
            """)

            self.cursor.execute("DROP TABLE messages")

            self.cursor.execute("ALTER TABLE messages_temp RENAME TO messages")

            self.conn.commit()

        except Exception as e:
            print(f"Error sorting messages table: {e}")
            self.conn.rollback()

    def trans(self):
        print(f"transing {self.input_dir_} to {self.output_dir_} \n")
        try:
            self.create_output_dir()
            for input_dir in self.input_dir_:
                self.parse_yaml(input_dir)
            self.insert_topics_table()
            
            for input_dir in self.input_dir_:
                single_bag_trans = SingleBagTrans(
                    input_dir, 
                    self.output_dir_, 
                    self.conn, 
                    self.cursor, 
                    self.message_count
                )
                single_bag_trans.trans_single_bag(self.topic_map)
                self.message_count = single_bag_trans.id
                self.starting_time_nanoseconds = single_bag_trans.starting_time_nanoseconds
                self.end_time_nanoseconds = single_bag_trans.end_time_nanoseconds

            self.sort_db_data()
            self.update_rosbag_yaml_data()
        finally:
            if self.db_manager:
                self.db_manager.close()

        print(f"transing {self.input_dir_} to {self.output_dir_} done\n")
