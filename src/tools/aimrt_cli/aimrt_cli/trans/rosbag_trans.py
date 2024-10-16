# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

from aimrt_cli.trans import TransBase
import os
import sqlite3
import shutil
from pathlib import Path
import yaml
from dataclasses import dataclass
# import ros2_plugin_proto.msg._ros_msg_wrapper as _ros_msg_wrapper
from aimrt_cli.trans.msg_wrapper import RosMsgWrapper





class IndentDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(IndentDumper, self).increase_indent(flow, False)

class SingleBagProcess:
    def __init__(self,topic_info_dict: dict, db_path: Path):        
        self.message_count = 0
        self.duration_nanoseconds = 0
        self.starting_time_nanoseconds = 100000000000000000000
                                         
        self.topic_with_message_count = {}
        self.topic_info_dict = topic_info_dict
        self.db_path = db_path
        self.get_info()
    
    def get_bag_info(self,conn, cursor):
        try:
            cursor.execute("SELECT topic_id, timestamp FROM messages")
            rows = cursor.fetchall()
            rows.sort()
            
            self.starting_time_nanoseconds = min(self.starting_time_nanoseconds, rows[0][1])
            self.duration_nanoseconds = rows[-1][1] - self.starting_time_nanoseconds
            self.message_count = len(rows)
            for row in rows:
                self.topic_with_message_count[self.topic_info_dict[row[0]].topic_name] = self.topic_with_message_count.get(self.topic_info_dict[row[0]].topic_name, 0) + 1
        except Exception as e:
            print(f"Error getting single bag info: {e}")
            conn.rollback()
            
    def get_info(self):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        self.get_bag_info(conn, cursor)
        
@dataclass
class TopicInfo:
    topic_id: int
    topic_name: str
    msg_type: str
    serialization_type: str
    

class RosbagTrans(TransBase):
    def __init__(self, input_dir: str, output_dir: str):
        super().__init__(output_dir)
        self.input_dir_ = input_dir
        self.topics_list = {}
        self.topic_info_dict = {}
        self.files_list = {}
        self.bag_info_list = []
        self.message_count = 0
        self.all_duration = 0
        self.topic_with_message_count = {}
        self.starting_time_nanoseconds = 100000000000000000000
        self.duration_nanoseconds = 0

        self.rosbag_yaml_data = {
            "version": 5,
            "storage_identifier": "sqlite3",
            "duration": {
                "nanoseconds": 0
            },
            "starting_time": {
                "nanoseconds_since_epoch": 0
            },
            "message_count": 0,
            "topics_with_message_count": [],
            "compression_format": "",
            "compression_mode": "",
            "relative_file_paths": [],
            "files": []
        }
        
    def copy_file(self):
        if os.path.exists(self.output_dir_):
            shutil.rmtree(self.output_dir_)
        try:
            shutil.copytree(self.input_dir_, self.output_dir_)
            print(f"目录已成功从 {self.input_dir_} 复制到 {self.output_dir_}")
        except shutil.Error as e:
            print(f"复制错误: {e}")
        except OSError as e:
            print(f"系统错误: {e}")
        
    def parse_yaml(self):
        with open(os.path.join(self.output_dir_, "metadata.yaml"), "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        if data["aimrt_bagfile_information"] is not None:
            if data["aimrt_bagfile_information"]["topics"] is not None:
                self.topics_list = data["aimrt_bagfile_information"]["topics"]
                for topic in self.topics_list:
                    self.topic_info_dict[topic["id"]] = TopicInfo(topic["id"], topic["topic_name"], topic["msg_type"], topic["serialization_type"])                
            else:
                raise Exception("No topics found in metadata.yaml")

            if data["aimrt_bagfile_information"]["files"] is not None:
                self.files_list = data["aimrt_bagfile_information"]["files"]
            else:
                raise Exception("No files found in metadata.yaml")
        else:
            raise Exception("No aimrt_bagfile_information found in metadata.yaml")
        
        self.rosbag_yaml_data = {
            "version": 5,
            "storage_identifier": "sqlite3",
        }
        return data
    
    def update_rosbag_yaml(self):
        self.rosbag_yaml_data = {
            "version": 5,
            "storage_identifier": "sqlite3",
            "duration": {
                "nanoseconds": self.all_duration  
            },
            "starting_time": {
                "nanoseconds_since_epoch": self.starting_time_nanoseconds
            },
            "message_count": self.message_count,  
            "topics_with_message_count": [],
            "compression_format": "",
            "compression_mode": "",
            "relative_file_paths": [],
            "files": [],
        }
        
        for topic in self.topics_list:
            topic_message_count = self.topic_with_message_count.get(topic["topic_name"], 0)            
            topic_entry = {
                "topic_metadata": {
                    "name": topic["topic_name"],
                    "type": topic["msg_type"].replace('ros2:', ''),
                    "serialization_format": "cdr",
                    "offered_qos_profiles": self.format_qos_profiles()
                },
                "message_count": topic_message_count
            }
            self.rosbag_yaml_data["topics_with_message_count"].append(topic_entry)

        for file_info in self.bag_info_list:
            self.rosbag_yaml_data["relative_file_paths"].append(file_info.db_path.name)
            file_entry = {
                "path": file_info.db_path.name,
                "starting_time": {
                    "nanoseconds_since_epoch": file_info.starting_time_nanoseconds
                },
                "duration": {
                    "nanoseconds": file_info.duration_nanoseconds
                },
                "message_count": file_info.message_count
            }
            self.rosbag_yaml_data["files"].append(file_entry)
        
        final_yaml_data = {
            "rosbag2_bagfile_information": self.rosbag_yaml_data
        }
        

        with open(os.path.join(self.output_dir_, "metadata.yaml"), "w") as f:
            yaml_str = yaml.dump(final_yaml_data, Dumper=IndentDumper, default_flow_style=False, sort_keys=False,indent=2,width=1000000)
            yaml_str = yaml_str.replace("\'", "\"")
            f.write(yaml_str)
        print("yaml update success")

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
        qos_string = '- history: {history}\\n  depth: {depth}\\n  reliability: {reliability}\\n  durability: {durability}\\n  deadline:\\n    sec: {deadline[sec]}\\n    nsec: {deadline[nsec]}\\n  lifespan:\\n    sec: {lifespan[sec]}\\n    nsec: {lifespan[nsec]}\\n  liveliness: {liveliness}\\n  liveliness_lease_duration:\\n    sec: {liveliness_lease_duration[sec]}\\n    nsec: {liveliness_lease_duration[nsec]}\\n  avoid_ros_namespace_conventions: {avoid_ros_namespace_conventions}'.format(**qos_dict).replace('True', 'true').replace('False', 'false')

        
        return qos_string
    
    def update_messages_table(self, conn, cursor):
        try:
            cursor.execute("UPDATE messages SET topic_id = topic_id + 1")
            conn.commit()
        except Exception as e:
            print(f"Error updating messages table: {e}")
            conn.rollback()
    
    def insert_topics_table(self, conn, cursor):
        try:
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS "topics" (
                "id"    INTEGER,
                "name"  TEXT NOT NULL,
                "type"  TEXT NOT NULL,
                "serialization_format"   TEXT NOT NULL,
                "offered_qos_profiles"   TEXT NOT NULL,
                PRIMARY KEY("id")
            )
        """)
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
            qos_json = yaml.dump(qos_dict,Dumper=IndentDumper,sort_keys=False)

            # 从self.topics_list填充topics表
            for topic in self.topics_list:            
                topic['offered_qos_profiles'] = self.format_qos_profiles()
                cursor.execute("""
                INSERT INTO topics (id, name, type, serialization_format, offered_qos_profiles)
                VALUES (?, ?, ?, ?, ?)
                """, (
                    topic['id'] + 1,
                    topic['topic_name'],
                    topic['msg_type'].replace('ros2:', ''),
                    'cdr',  # 默认使用'cdr'作为序列化格式
                    qos_json
                ))
            conn.commit()
        except Exception as e:
            print(f"Error updating topics table: {e}")
            conn.rollback()
    def insert_schema_version(self, conn, cursor):
        try:
            cursor.execute("""
            CREATE TABLE "schema" (
                "schema_version"	INTEGER,
                "ros_distro"	TEXT NOT NULL,
                PRIMARY KEY("schema_version")
            );
            """)
            cursor.execute("""
            INSERT INTO schema (schema_version, ros_distro)
            VALUES (?, ?)
            """, (3, "humble"))
            conn.commit()
        except Exception as e:
            print(f"Error updating schema version: {e}")
            conn.rollback()

    def insert_metadata_table(self, conn, cursor):
        try:
            cursor.execute("""
            CREATE TABLE "metadata" (
                "id"	INTEGER,
                "metadata_version"	INTEGER NOT NULL,
                "metadata"	TEXT NOT NULL,
                PRIMARY KEY("id")
            );
            """)
        except Exception as e:
            print(f"Error updating metadata table: {e}")
            conn.rollback()
    
    def update_pb_msg(self, conn, cursor):
        
        print(os.path.abspath(__file__))

        def encode_topic_name(topic_name : str, msg_type : str):
            return topic_name.replace('/', '_2F') + '/' + msg_type.replace('/', '_2F').replace(':', '_3A').replace('.', '_2E')
        
        # try:
        cursor.execute("SELECT topic_id, timestamp, data FROM messages")
        rows = cursor.fetchall()
        for row in rows:
            topic_id, timestamp, data = row
            if topic_id not in self.topic_info_dict or not self.topic_info_dict[topic_id].msg_type.startswith("pb"):
                continue                
            
            if data is None:
                print(f"Warning: data is None for topic_id {topic_id}")
                continue
        
            if not isinstance(data, (bytes, bytearray)):
                print(f"Warning: data is not bytes, it's {type(data)}. Attempting to convert...")
                try:
                    if isinstance(data, str):
                        data = data.encode('utf-8')
                    else:
                        data = bytes(data)
                except Exception as e:
                    print(f"Failed to convert data to bytes: {e}")
                    continue
            # print(data)
            wrapper = RosMsgWrapper()
            wrapper.serialization_type = "pb"
            wrapper.__setattr__("_data", data)
            wrapper.context = [self.topic_info_dict[topic_id].msg_type]
            print("succeed")
            
            cursor.execute("UPDATE messages SET data = ? WHERE topic_id = ?", (wrapper.data, topic_id))
        conn.commit()
        print("update pb msg success")
        # except Exception as e:
        #     print(f"Error updating messages table: {e}")
        #     # conn.rollback()

        # try:
        cursor.execute("SELECT id, name, type FROM topics")
        rows = cursor.fetchall()
        for row in rows:
            id = row[0]
            topic_name = row[1]
            msg_type = row[2]
            if msg_type.startswith("pb"):
                topic_name = encode_topic_name(topic_name, msg_type)
                msg_type = "pb"      
            cursor.execute("UPDATE topics SET name = ? , type = ? WHERE id = ?", (topic_name, msg_type, id))
        conn.commit()
        # except Exception as e:
        #     print(f"Error updating messages table: {e}")
        #     conn.rollback()
                            
    def trans_single_db(self, db_path: Path):
        single_bag_info = SingleBagProcess(self.topic_info_dict, db_path)
        self.all_duration += single_bag_info.duration_nanoseconds
        self.message_count += single_bag_info.message_count
        self.starting_time_nanoseconds = min(self.starting_time_nanoseconds, single_bag_info.starting_time_nanoseconds)
        
        for topic in single_bag_info.topic_with_message_count:
            self.topic_with_message_count[topic] = self.topic_with_message_count.get(topic, 0) + single_bag_info.topic_with_message_count[topic]
        self.bag_info_list.append(single_bag_info)
        
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        # try:            
        self.insert_schema_version(conn, cursor)
        self.insert_metadata_table(conn, cursor)
        self.insert_topics_table(conn, cursor)
        self.update_messages_table(conn, cursor)
        self.update_pb_msg(conn, cursor)
        # except Exception as e:
        #     print(f"Error updating messages table: {e}")
        #     conn.rollback()
        
    def trans(self):
        self.copy_file()
        self.parse_yaml()
        for db_path in self.files_list:
            trans_path = Path(self.output_dir_) / db_path['path']
            self.trans_single_db(trans_path)
            break
        self.update_rosbag_yaml()
        
    