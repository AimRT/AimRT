import subprocess
import shutil
import os


def repair_bag(bag_path: str):
    try:
        journal_files = [f for f in os.listdir(bag_path) if f.endswith("-journal")]
        if not journal_files:
            print(f"there is no journal file in  {bag_path}")
            return

        print(f"detect {len(journal_files)} journal files, start to repair")

        for journal_file in journal_files:
            journal_file_name = journal_file.split("-")[0]
            journal_path = os.path.join(bag_path, journal_file_name)
            print(f"journal_path: {journal_path}")

            backup_path = f"{journal_path}.bak"
            recover_path = os.path.join(bag_path, "recovered.db3")
            shutil.copy2(journal_path, backup_path)

            try:
                cmd = f'sqlite3 "{journal_path}" ".recover" | sqlite3 "{recover_path}"'
                print(f"cmd: {cmd}")
                subprocess.run(cmd, shell=True, check=True)

                if os.path.exists(recover_path):
                    os.replace(recover_path, journal_path)
                    print(f"{journal_path} repair done \n")

            except subprocess.CalledProcessError as e:
                print(f"database repair failed: {e.stderr.decode()}")
            finally:
                if os.path.exists(backup_path):
                    os.remove(backup_path)

    except Exception as e:
        print(f"repair failed: {str(e)}\n")
