# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse
import signal
import subprocess
import sys
import tempfile
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
from multiprocessing import Process, Queue
from typing import Dict, List, Tuple

from example_items import *


class ExampleRunner:
    def __init__(self):
        # initial operations
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"\n{YELLOW}{BOLD}Running all examples on {now}...{RESET}\n")

        # initial member variables
        self.args = self.parse_args()  # parse command line arguments
        self.test_start_time = time.time()
        self.max_threads = self.args.parallel_num  # maximum number of threads to use
        self.item_results = {}  # store test results for each item
        self.lock_dict = {}  # store lock for some limisted items

        if self.args.save is not None:
            self.check_and_create_directory(self.args.save)  # todo ...

        # Read version from VERSION file
        version_file = os.path.join(default_build_path, "..", "VERSION")
        with open(version_file, "r") as f:
            version = f.read().strip()

        subprocess.run(["pip3",
                        "install",
                        f"./aimrt_py_pkg/dist/aimrt_py-{version}-cp310-cp310-linux_x86_64.whl",
                        "--force-reinstall"],
                       cwd=default_build_path,
                       )
        subprocess.run(
            ["bash", os.path.join("build_examples_py_pb_rpc.sh")],
            cwd=os.path.join(py_cwd, "pb_rpc"),
        )
        subprocess.run(
            ["bash", os.path.join("build_examples_py_pb_chn.sh")],
            cwd=os.path.join(py_cwd, "pb_chn"),
        )
        subprocess.run(
            ["bash", os.path.join("build_examples_py_ros2_rpc.sh")],
            cwd=os.path.join(py_cwd, "ros2_rpc"),
        )

    def check_and_create_directory(self, test_log_save_path: str) -> None:
        if test_log_save_path and not os.path.exists(test_log_save_path):
            os.makedirs(test_log_save_path)

    def update_progress(self, progress):
        sys.stdout.write("\r" + self.draw_progress_bar(progress))
        sys.stdout.flush()

    def draw_progress_bar(self, progress: float, total_width: int = 50) -> str:
        progress = max(0, min(1, progress))

        completed_width = int(total_width * progress)

        bar = "█" * completed_width + "-" * (total_width - completed_width)

        percent = progress * 100

        return f"{BRIGHT_GREEN}{percent:.1f}% [{bar}]{RESET}"

    def parse_args(self):
        parser = argparse.ArgumentParser(description="Run Python tests")
        parser.add_argument("-p", "--print-output", action="store_true", help="Print test output", default=False)
        parser.add_argument("-t", "--test", nargs="+", type=str, help="Test name", default=["all"])
        parser.add_argument("-i", "--ignore", nargs="+", type=str, help="Ignore test name", default=None)
        parser.add_argument("-s", "--save", nargs="?", default=None, const=default_save_path, help="Save test log.")
        parser.add_argument("-n", "--parallel_num", type=int, help="Number of parallel tests", default=10)

        args = parser.parse_args()
        return args

    def generate_test_report(self, test_results: Dict[str, TestResult]) -> str:
        total_tests = len(test_results)
        successful_tests = sum(1 for result in test_results.values() if result == TestResult.SUCCESS)
        not_found_tests = sum(1 for result in test_results.values() if result == TestResult.EXPECTED_OUTPUT_NOT_FOUND)
        forbidden_tests = sum(1 for result in test_results.values() if result == TestResult.FORBIDDEN_OUTPUT_FOUND)
        exit_failed_tests = sum(1 for result in test_results.values() if result == TestResult.EXIT_STRING_NOT_FOUND)
        not_run_tests = sum(1 for result in test_results.values() if result is None)

        width = 65
        report = f"""
{CYAN}{BOLD}
  _____         _     ____                       _
 |_   _|__  ___| |_  |  _ \\ ___ _ __   ___  _ __| |_ _
   | |/ _ \\/ __| __| | |_) / _ \\ '_ \\ / _ \\| '__| __(_)
   | |  __/\\__ \\ |_  |  _ <  __/ |_) | (_) | |  | |_ _
   |_|\\___||___/\\__| |_| \\_\\___| .__/ \\___/|_|   \\__(_)
                               |_|
{RESET}
{YELLOW}{BOLD}► Overall Result:{RESET}
{WHITE}{'Total tests:':┈<{width}}{CYAN}{total_tests}
{GREEN}{'Successful tests:':┈<{width-4}}{successful_tests}
{RED}{'Failed tests:':┈<{width-4}}{not_found_tests + exit_failed_tests + forbidden_tests}
    {YELLOW}{'• Expected Output Not Found:':┈<{width-12}}{not_found_tests}
    {MAGENTA}{'• Forbidden Output Found:':┈<{width-12}}{forbidden_tests}
    {RED}{'• Exit String Not Found:':┈<{width-12}}{exit_failed_tests}
{BLUE}{'Not run tests:':┈<{width-4}}{not_run_tests}

{YELLOW}{BOLD}► Detailed Results:{RESET}
"""
        for test_name, result in test_results.items():
            if result == TestResult.SUCCESS:
                status = f"{GREEN}✔ Success{RESET}"
            elif result == TestResult.EXPECTED_OUTPUT_NOT_FOUND:
                status = f"{YELLOW}⚠ Expected Output Not Found{RESET}"
            elif result == TestResult.FORBIDDEN_OUTPUT_FOUND:
                status = f"{MAGENTA}✘ Forbidden Output Found{RESET}"
            elif result == TestResult.EXIT_STRING_NOT_FOUND:
                status = f"{RED}☹ Exit String Not Found{RESET}"
            else:  # result is None
                status = f"{BLUE}- Not Run{RESET}"
            report += f"    {CYAN}•{RESET} {test_name:<65} {status}\n"

        success_rate = (
            (successful_tests / (total_tests - not_run_tests)) * 100 if (total_tests - not_run_tests) > 0 else 0
        )
        overall_success_rate = (successful_tests / total_tests) * 100 if total_tests > 0 else 0
        report += f"\n{YELLOW} Success Rate (excluding not run): {WHITE}{success_rate:.2f}%{RESET}"
        report += f"\n{YELLOW} Overall Success Rate: {WHITE}{overall_success_rate:.2f}%{RESET}"

        return report

    def check_item_format(self, item: dict) -> None:
        task_num_for_current_item = 0
        # check and some default parameters to be added
        if "script_path" not in item:
            raise ValueError("script_path is required in item")
        else:
            task_num_for_current_item = len(item["script_path"])

        if "expected_outputs" not in item:
            raise ValueError("expected_outputs is required in item")
        else:
            if len(item["expected_outputs"]) != task_num_for_current_item:
                raise ValueError("expected_outputs should have the same length as script_path")

        if "forbidden_outputs" not in item:
            item["forbidden_outputs"] = [default_forbidden_outputs] * task_num_for_current_item
        else:
            if len(item["forbidden_outputs"]) != task_num_for_current_item:
                raise ValueError("forbidden_outputs should have the same length as script_path")

        if "exit_string" not in item:
            item["exit_string"] = [default_exit_string] * task_num_for_current_item
        else:
            if len(item["exit_string"]) != task_num_for_current_item:
                raise ValueError("exit_string should have the same length as script_path")

        if "timeout" not in item:
            item["timeout"] = default_timeout

        # add default parameters:build directory path, if not exist
        if "cwd" not in item:
            item["cwd"] = default_build_path

        if "limit" in item and item["limit"] not in self.lock_dict:
            self.lock_dict[item["limit"]] = threading.Lock()

    def run_task_with_timeout(self, script_path: str, cwd: str, running_sec: int, wait_sec: int = 5) -> Tuple[str, str]:
        with tempfile.NamedTemporaryFile(mode="w+", encoding="latin-1", suffix=".log", delete=False) as temp_file:
            process = subprocess.Popen(
                script_path,
                stdout=temp_file,
                stderr=temp_file,
                text=True,
                shell=True,
                cwd=cwd,
                bufsize=1,
                universal_newlines=True,
                preexec_fn=os.setsid,
            )
            try:
                process.wait(timeout=running_sec)
            except subprocess.TimeoutExpired:
                # Send SIGTERM signal to the process group
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                # Wait for a short time to allow the process to terminate
                time.sleep(wait_sec)
                if process.poll() is None:
                    # If the process still hasn't terminated, forcefully terminate it
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)

            # Ensure the process has ended
            process.wait()

            # Flush the file buffer
            temp_file.flush()
            os.fsync(temp_file.fileno())

            # Read the log file content
            temp_file.seek(0)
            log_content = temp_file.read()

        os.remove(temp_file.name)

        return log_content

    def check_result(
        self,
        log_content: str,
        expected_outputs: List[str],
        forbidden_outputs: List[str],
        exit_str: str,
    ) -> TestResult:
        # prepare expected outputs
        expected_outputs_lines = []
        for expected_output in expected_outputs:
            expected_outputs_lines.extend(expected_output.splitlines())

        log_lines = log_content.splitlines()
        expected_index = 0
        for log_line in log_lines:
            if expected_index < len(expected_outputs_lines) and expected_outputs_lines[expected_index] in log_line:
                expected_index += 1

        # check expected outputs
        all_expected_found = expected_index == len(expected_outputs_lines)
        if not all_expected_found:
            # print(f"{RED}Expected outputs not found: {expected_outputs_lines[expected_index:]}{RESET}")
            return TestResult.EXPECTED_OUTPUT_NOT_FOUND

        # check forbidden outputs
        forbidden_found = any(forbidden_output in log_content for forbidden_output in forbidden_outputs)
        if forbidden_found:
            # print(f"{RED}Forbidden output found in log content{RESET}")
            return TestResult.FORBIDDEN_OUTPUT_FOUND

        # check exit string
        exit_found = exit_str in log_content
        if not exit_found:
            # print(f"{RED}Exit string not found: {exit_str}{RESET}")
            return TestResult.EXIT_STRING_NOT_FOUND

        return TestResult.SUCCESS

    def run_and_store_task_result(self, script_path: str, timeout: int, cwd: str, queue: Queue):
        log_content = self.run_task_with_timeout(script_path, cwd, timeout)
        queue.put((script_path, log_content))

    def find_element_index(self, lst, element):
        try:
            index = lst.index(element)
            return index
        except ValueError:
            return -1

    def run_item(self, item):
        self.check_item_format(item)
        process_list = []
        output_queue = Queue()

        # if has limits, then acquire lock before running
        if "limit" in item and item["limit"] in self.lock_dict:
            self.lock_dict[item["limit"]].acquire()

        for i in range(len(item["script_path"])):
            dt = (
                len(item["script_path"]) - 1 - i
            ) * 2  # this is to ensure sub / srv is close afert pub / cli (todo: better way to do this)
            self.item_results[item["script_path"][i]] = None
            process = Process(
                target=self.run_and_store_task_result,
                args=(item["script_path"][i], item["timeout"] + dt, item["cwd"], output_queue),
            )
            process.start()
            process_list.append(process)
            time.sleep(0.1)

        # wait for all processes to complete
        for process in process_list:
            process.join()

        # when all processes are done, release lock if has limits
        if "limit" in item and item["limit"] in self.lock_dict:
            self.lock_dict[item["limit"]].release()

        # from msg queue to get output
        result_dict = {}
        while not output_queue.empty():
            script_path, log_content = output_queue.get()
            result_dict[script_path] = log_content
            if self.args.print_output:
                print(f"\n{CYAN}{BOLD}Output of {script_path}:{RESET}\n{log_content}")
        for script_path, log_content in result_dict.items():
            idx = self.find_element_index(item["script_path"], script_path)
            self.item_results[script_path] = self.check_result(
                log_content, item["expected_outputs"][idx], item["forbidden_outputs"][idx], item["exit_string"][idx]
            )
            # if need to save log when test failed, save it
            if self.args.save and self.item_results[script_path] != TestResult.SUCCESS:
                with open(f"{self.args.save}/{os.path.basename(script_path)}.log", "w") as f:
                    f.write(">> **expected outputs:**\n" + "\n".join(item["expected_outputs"][idx]))
                    f.write("\n\n>> **forbidden_outputs:**\n" + "\n".join(item["forbidden_outputs"][idx]))
                    f.write("\n\n>> **running log:**\n")
                    f.write(log_content)

    def run(self) -> None:
        has_running_num = 0
        # run all examples in parallel
        with ThreadPoolExecutor(max_workers=self.max_threads) as executor:
            futures = {}  # store all futures to wait for completion
            for idx, test_item in enumerate(test_items):

                # filter out examples  which are not desired to run
                if "tags" in test_item:
                    if not all(item in test_item["tags"] for item in self.args.test):
                        continue

                    if self.args.ignore and any(item in test_item["tags"] for item in self.args.ignore):
                        continue

                futures[
                    executor.submit(
                        self.run_item,
                        item=test_item,
                    )
                ] = idx
                has_running_num += 1

            # wait for all tests to complete
            finished_num = 0
            self.update_progress(0)
            for future in as_completed(futures):
                if has_running_num == 0:
                    break
                finished_num += 1
                self.update_progress(finished_num / has_running_num)
                time.sleep(0.1)
                future.result()

    def __del__(self):
        # print test report
        report = self.generate_test_report(self.item_results)
        print(report)

        # calculate total running time
        print(f"\n{YELLOW}{BOLD}Test all examples finished, consuming {time.time() - self.test_start_time:.2f}s{RESET}")


if __name__ == "__main__":
    ExampleRunner().run()
