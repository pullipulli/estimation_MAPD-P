# Use the gen_some_results arguments to set the variable parameters (This notebook works with the assumption that there is only one variable parameter at a time)
# Number arguments (generate n evenly spaced parameter numbers): -agents n, -tasks n, -tasks_frequency n, -starts n, -goals n
# Number arguments (generate n evenly spaced parameter numbers): -agents_list <args>, -tasks_list <args>, -tasks_frequency_list <args>, -starts_list <args>, -goals_list <args>
# This cell will take some time to run (obviously)
import subprocess
from multiprocessing import Pool
from termcolor import colored

def run_script(*args):
    try:
        subprocess.run(["python", "-m", 'gen_some_results', *args])
    except Exception as e:
        print(colored(f"Process {args[0]} failed", "red"))
        print(e)
    else:
        print(colored(f"Process {args[0]} finished", "cyan"))

if __name__ == '__main__':
    pool = Pool(processes=6)

    pool.apply_async(run_script, ["-agents", '4'])
    print(colored("Process agents started", "green"))
    pool.apply_async(run_script, ["-tasks_list", '15', '30', '90'])
    print(colored("Process tasks started", "green"))
    pool.apply_async(run_script, ["-tasks_frequency_list", '0.2', '0.6', '2'])
    print(colored("Process tasks_frequency started", "green"))
    pool.apply_async(run_script, ["-starts", '3'])
    print(colored("Process starts started", "green"))
    pool.apply_async(run_script, ["-goals", '3'])
    print(colored("Process goals started", "green"))
    pool.apply_async(run_script, ["-td_update_list", '1', '10', '20'])
    print(colored("Process td_update started", "green"))

    print("All Processes started")

    pool.close()
    pool.join()
    print("All Processes finished")

