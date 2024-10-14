import argparse
import os
import signal
import subprocess
import tempfile
import time
from multiprocessing import Process, Queue
from typing import Dict, List, Tuple


class TestResult:
    SUCCESS = 0
    EXPECTED_OUTPUT_NOT_FOUND = 1
    FORBIDDEN_OUTPUT_FOUND = 2
    EXIT_STRING_NOT_FOUND = 3


def run_program_with_timeout(script_path: str, timeout_seconds: int) -> Tuple[str, str]:
    with tempfile.NamedTemporaryFile(mode='w+', suffix='.log', delete=False) as temp_file:
        process = subprocess.Popen(script_path, stdout=temp_file, stderr=temp_file,
                                   text=True, shell=True, cwd=os.path.dirname(script_path),
                                   bufsize=1, universal_newlines=True, preexec_fn=os.setsid)

        try:
            process.wait(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            # Send SIGTERM signal to the process group
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            # Wait for a short time to allow the process to terminate
            time.sleep(1)
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


def run_and_capture(script_path: str, timeout: int, queue: Queue, program_name: str) -> None:
    log_content = run_program_with_timeout(script_path, timeout)
    queue.put((program_name, log_content))


def check_result(log_content: str, expected_outputs: List[str], forbidden_outputs: List[str]) -> TestResult:
    RED = "\033[91m"
    RESET = "\033[0m"

    # prepare expected outputs
    expected_outputs_lines = []
    for expected_output in expected_outputs:
        expected_outputs_lines.extend(expected_output.splitlines())

    log_lines = log_content.splitlines()
    expected_index = 0
    for log_line in log_lines:
        if expected_index < len(expected_outputs_lines) and expected_outputs_lines[expected_index] in log_line:
            expected_index += 1
    all_expected_found = expected_index == len(expected_outputs_lines)

    if not all_expected_found:
        print(f"{RED}Expected outputs not found: {expected_outputs_lines[expected_index:]}{RESET}")
        return TestResult.EXPECTED_OUTPUT_NOT_FOUND

    forbidden_found = any(forbidden_output in log_content for forbidden_output in forbidden_outputs)
    if forbidden_found:
        print(f"{RED}Forbidden output found in log content{RESET}")
        return TestResult.FORBIDDEN_OUTPUT_FOUND

    exit_str = "AimRT exit."
    exit_found = exit_str in log_content

    if not exit_found:
        print(f"{RED}Exit string not found: {exit_str}{RESET}")
        return TestResult.EXIT_STRING_NOT_FOUND

    return TestResult.SUCCESS


def single_test(script_path: str,
                expected_outputs: List[str],
                forbidden_outputs: List[str],
                timeout: int,
                print_output: bool = False) -> TestResult:
    start_time = time.perf_counter()

    print("\n" + "=" * 40)
    print(f"Running test: {os.path.basename(script_path)}")

    log_content = run_program_with_timeout(script_path, timeout)

    if print_output:
        print(log_content)

    end_time = time.perf_counter()
    print(f"Total time taken: {end_time - start_time:.2f} seconds")

    result = check_result(log_content, expected_outputs, forbidden_outputs)

    print("=" * 40 + "\n")

    return result


def paired_test(server_script: str,
                client_script: str,
                server_expected_outputs: List[str],
                client_expected_outputs: List[str],
                server_forbidden_outputs: List[str],
                client_forbidden_outputs: List[str],
                timeout: int,
                print_server_output: bool = False,
                print_client_output: bool = False) -> Tuple[TestResult, TestResult]:
    start_time = time.perf_counter()

    print("\n" + "=" * 40)
    print(f"Running paired test: {os.path.basename(server_script)} and\n" +
          f"                     {os.path.basename(client_script)}")
    output_queue = Queue()

    server_process = Process(target=run_and_capture, args=(server_script, timeout, output_queue, "Server"))
    client_process = Process(target=run_and_capture, args=(client_script, timeout, output_queue, "Client"))

    server_process.start()
    client_process.start()

    server_process.join()
    client_process.join()

    end_time = time.perf_counter()
    print(f"Total time taken: {end_time - start_time:.2f} seconds")

    while not output_queue.empty():
        program_name, log_content = output_queue.get()
        if program_name == "Server":
            server_log_content = log_content
            if print_server_output:
                print(f"\nServer Output:")
                print(log_content)
        elif program_name == "Client":
            client_log_content = log_content
            if print_client_output:
                print(f"\nClient Output:")
                print(log_content)

    server_result = check_result(server_log_content, server_expected_outputs, server_forbidden_outputs)
    client_result = check_result(client_log_content, client_expected_outputs, client_forbidden_outputs)

    print("=" * 40 + "\n")

    return server_result, client_result


def generate_test_report(test_results: Dict[str, TestResult]) -> str:
    # ANSI escape sequences for colors
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"

    total_tests = len(test_results)
    successful_tests = sum(1 for result in test_results.values() if result == TestResult.SUCCESS)
    not_found_tests = sum(1 for result in test_results.values() if result == TestResult.EXPECTED_OUTPUT_NOT_FOUND)
    forbidden_tests = sum(1 for result in test_results.values() if result == TestResult.FORBIDDEN_OUTPUT_FOUND)
    exit_failed_tests = sum(1 for result in test_results.values() if result == TestResult.EXIT_STRING_NOT_FOUND)
    not_run_tests = sum(1 for result in test_results.values() if result is None)

    report = f"""
{CYAN}{BOLD}╔═══════════════════════════════════════════╗
║             Test Report Summary            ║
╚═══════════════════════════════════════════╝{RESET}

{WHITE}Total tests:         {CYAN}{total_tests}
{GREEN}Successful tests:    {successful_tests}
{RED}Failed tests:        {not_found_tests + exit_failed_tests}
  {YELLOW}• Expected Output Not Found:        {not_found_tests}
  {MAGENTA}• Forbidden Output Found:           {forbidden_tests}
  {RED}• Exit String Not Found:            {exit_failed_tests}
{BLUE}Not run tests:                        {not_run_tests}

{YELLOW}{BOLD}Detailed Results:{RESET}
"""
    for test_name, result in test_results.items():
        if result == TestResult.SUCCESS:
            status = f"{GREEN}✔ Success{RESET}"
        elif result == TestResult.EXPECTED_OUTPUT_NOT_FOUND:
            status = f"{YELLOW}⚠ Expected Output Not Found{RESET}"
        elif result == TestResult.FORBIDDEN_OUTPUT_FOUND:
            status = f"{MAGENTA}✘ Forbidden Output Found{RESET}"
        elif result == TestResult.EXIT_STRING_NOT_FOUND:
            status = f"{RED}✘ Exit String Not Found{RESET}"
        else:  # result is None
            status = f"{BLUE}- Not Run{RESET}"
        report += f"  {CYAN}•{RESET} {test_name:<20} {status}\n"

    success_rate = (successful_tests / (total_tests - not_run_tests)) * 100 if (total_tests - not_run_tests) > 0 else 0
    overall_success_rate = (successful_tests / total_tests) * 100 if total_tests > 0 else 0
    report += f"\n{YELLOW}Success Rate (excluding not run): {WHITE}{success_rate:.2f}%{RESET}"
    report += f"\n{YELLOW}Overall Success Rate: {WHITE}{overall_success_rate:.2f}%{RESET}"

    return report


def parse_args():
    parser = argparse.ArgumentParser(description="Run Python tests")
    parser.add_argument("-p", "--print-output", action="store_true", help="Print test output", default=False)
    parser.add_argument("-t", "--test", type=str, help="Test name", default="all")
    args = parser.parse_args()
    return args


if __name__ == "__main__":
    args = parse_args()
    try:
        test_results = {"app mode": None, "registration mode": None,
                        "http rpc server": None, "http rpc client": None,
                        "grpc rpc server": None, "grpc rpc client": None,
                        "ros2 rpc server": None, "ros2 rpc client": None,
                        "http channel sub": None, "http channel pub": None,
                        "ros2 channel sub": None, "ros2 channel pub": None,
                        "parameter": None}

        common_forbidden_outputs = ["[Error]", "[Fatal]", "Traceback (most recent call last):"]

        # build pb_rpc and pb_chn
        if args.test in ["all", "rpc"] or "rpc" in args.test:
            subprocess.run(["bash", os.path.join(os.getcwd(), "pb_rpc", "build_examples_py_pb_rpc.sh")],
                           cwd=os.path.join(os.getcwd(), "pb_rpc"))
        if args.test in ["all", "channel"] or "channel" in args.test:
            subprocess.run(["bash", os.path.join(os.getcwd(), "pb_chn", "build_examples_py_pb_chn.sh")],
                           cwd=os.path.join(os.getcwd(), "pb_chn"))

        if args.test in ["all", "app mode", "helloworld"]:
            test_results["app mode"] = single_test(
                os.path.join(os.getcwd(), "helloworld", "start_examples_py_helloworld_app_mode.sh"),
                expected_outputs=["{'key1': 'val1', 'key2': 'val2'}", "Loop count: 1", "Loop count: 2"],
                forbidden_outputs=common_forbidden_outputs,
                timeout=3,
                print_output=args.print_output)

        if args.test in ["all", "registration mode", "helloworld"]:
            test_results["registration mode"] = single_test(
                os.path.join(os.getcwd(), "helloworld", "start_examples_py_helloworld_registration_mode.sh"),
                expected_outputs=["{'key1': 'val1', 'key2': 'val2'}"] + ["run test task"] * 3,
                forbidden_outputs=common_forbidden_outputs,
                timeout=2,
                print_output=args.print_output)

        rpc_server_expected_outputs = ["Server handle new rpc call."]
        rpc_client_expected_outputs = ["Call rpc done, status: suc, code 0, msg: OK"]

        if args.test in ["all", "rpc", "http rpc"]:
            test_results["http rpc server"], test_results["http rpc client"] = paired_test(
                os.path.join(os.getcwd(), "pb_rpc", "start_examples_py_pb_rpc_http_server.sh"),
                os.path.join(os.getcwd(), "pb_rpc", "start_examples_py_pb_rpc_http_client.sh"),
                server_expected_outputs=rpc_server_expected_outputs,
                client_expected_outputs=rpc_client_expected_outputs,
                server_forbidden_outputs=common_forbidden_outputs,
                client_forbidden_outputs=common_forbidden_outputs,
                timeout=2,
                print_server_output=args.print_output,
                print_client_output=args.print_output)

        if args.test in ["all", "rpc", "grpc rpc"]:
            test_results["grpc rpc server"], test_results["grpc rpc client"] = paired_test(
                os.path.join(os.getcwd(), "pb_rpc", "start_examples_py_pb_rpc_grpc_server.sh"),
                os.path.join(os.getcwd(), "pb_rpc", "start_examples_py_pb_rpc_grpc_client.sh"),
                server_expected_outputs=rpc_server_expected_outputs,
                client_expected_outputs=rpc_client_expected_outputs,
                server_forbidden_outputs=common_forbidden_outputs,
                client_forbidden_outputs=common_forbidden_outputs,
                timeout=2,
                print_server_output=args.print_output,
                print_client_output=args.print_output)

        if args.test in ["all", "rpc", "ros2 rpc"]:
            test_results["ros2 rpc server"], test_results["ros2 rpc client"] = paired_test(
                os.path.join(os.getcwd(), "pb_rpc", "start_examples_py_pb_rpc_ros2_server.sh"),
                os.path.join(os.getcwd(), "pb_rpc", "start_examples_py_pb_rpc_ros2_client.sh"),
                server_expected_outputs=rpc_server_expected_outputs,
                client_expected_outputs=rpc_client_expected_outputs,
                server_forbidden_outputs=common_forbidden_outputs,
                client_forbidden_outputs=common_forbidden_outputs,
                timeout=2,
                print_server_output=args.print_output,
                print_client_output=args.print_output)

        channel_sub_expected_outputs = ["Get new pb event, data: {", "\"msg\": \"example msg\"", "\"num\": 123456"]
        channel_pub_expected_outputs = ["Publish new pb event, data: {", "\"msg\": \"example msg\"", "\"num\": 123456"]

        if args.test in ["all", "channel", "http channel"]:
            test_results["http channel sub"], test_results["http channel pub"] = paired_test(
                os.path.join(os.getcwd(), "pb_chn", "start_examples_py_pb_chn_http_sub.sh"),
                os.path.join(os.getcwd(), "pb_chn", "start_examples_py_pb_chn_http_pub.sh"),
                server_expected_outputs=channel_sub_expected_outputs,
                client_expected_outputs=channel_pub_expected_outputs,
                server_forbidden_outputs=common_forbidden_outputs,
                client_forbidden_outputs=common_forbidden_outputs,
                timeout=3,
                print_server_output=args.print_output,
                print_client_output=args.print_output)

        if args.test in ["all", "channel", "ros2 channel"]:
            test_results["ros2 channel sub"], test_results["ros2 channel pub"] = paired_test(
                os.path.join(os.getcwd(), "pb_chn", "start_examples_py_pb_chn_ros2_sub.sh"),
                os.path.join(os.getcwd(), "pb_chn", "start_examples_py_pb_chn_ros2_pub.sh"),
                server_expected_outputs=channel_sub_expected_outputs,
                client_expected_outputs=channel_pub_expected_outputs,
                server_forbidden_outputs=common_forbidden_outputs,
                client_forbidden_outputs=common_forbidden_outputs,
                timeout=3,
                print_server_output=args.print_output,
                print_client_output=args.print_output)

        if args.test in ["all", "parameter"]:
            test_results["parameter"] = single_test(
                os.path.join(os.getcwd(), "parameter", "start_examples_py_parameter.sh"),
                expected_outputs=["Start SetParameterLoop.",
                                  "SetParameterLoop count: 1 -------------------------",
                                  "Set parameter, key: 'key-1', val: 'val-1'",
                                  "Start GetParameterLoop.",
                                  "GetParameterLoop count: 1 -------------------------",
                                  "Get parameter, key: 'key-1', val: 'val-1'"],
                forbidden_outputs=common_forbidden_outputs,
                timeout=3,
                print_output=args.print_output)

    except KeyboardInterrupt:
        print("\nTests interrupted by user")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        report = generate_test_report(test_results)
        print(report)
