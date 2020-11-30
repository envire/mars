from __future__ import print_function
import subprocess, resource
import os
import time
from shutil import copyfile
from timeit import timeit
import signal


CONF_FOLDER = "/opt/workspace/simulation/mars/plugins/envire_mls/testMlsData/"
tests = ["debug.yml", "entrance_104cm.yml" , "entrance_248cm.yml", "tube_100cm.yml"]

def runTestNoTime(path_to_copy):
    if os.path.exists(CONF_FOLDER + "conf.yml"):
      os.remove(CONF_FOLDER + "conf.yml")
    copyfile(path_to_copy, CONF_FOLDER+"conf.yml")
    usage_start = resource.getrusage(resource.RUSAGE_CHILDREN)
    
    p1 = subprocess.Popen(['mars_app'], preexec_fn=os.setsid, shell=True, stderr=subprocess.PIPE, universal_newlines=True)
    #for stdout_line in iter(p1.stdout.readline, ""):
    #  #print(stdout_line)
    #  print("LALALA")
    for stderr_line in iter(p1.stderr.readline, ""):
      if "[EnvireMls::update] Goal was reached" in stderr_line:
        print(stderr_line)
        #p1.stdout.close()
        os.killpg(os.getpgid(p1.pid), signal.SIGTERM)
        print("Process killed")
      #yield stdout_line
      #  yield "**************************** GOAL REACHED! ****************************"
    #return_code = p1.wait()
    #if return_code:
    #  raise subprocess.CalledProcessError(return_code, 'mars_app')
    #(stdout, stderr) = p1.communicate()

    duration = 0
    usage_end = resource.getrusage(resource.RUSAGE_CHILDREN)
    cpu_time = usage_end.ru_utime - usage_start.ru_utime
    return (cpu_time, duration)



def runTest(path_to_copy):
    if os.path.exists(CONF_FOLDER + "conf.yml"):
      os.remove(CONF_FOLDER + "conf.yml")
    copyfile(path_to_copy, CONF_FOLDER+"conf.yml")
    usage_start = resource.getrusage(resource.RUSAGE_CHILDREN)
    duration = timeit(stmt = "subprocess.call('mars_app')", setup = "import subprocess", number = 1)
    usage_end = resource.getrusage(resource.RUSAGE_CHILDREN)
    cpu_time = usage_end.ru_utime - usage_start.ru_utime
    return (cpu_time, duration)

res = {}
for test in tests:
    #(cpu_time, real_time) = runTest(CONF_FOLDER+test)
    (cpu_time, real_time) = runTestNoTime(CONF_FOLDER+test)
    #for mess in runTestNoTime(CONF_FOLDER+test):
    #  if "[EnvireMls::update] Goal was reached" in mess:
    #    print("**************************** GOAL REACHED! ****************************")
    res[test] = {}
    res[test]["cpu_time"] = cpu_time
    res[test]["real_time"] = real_time
    raw_input("Press key to proceed with the next test")

print(res)

#{'conf_2.yml': (197.17293199999997, 134.77659487724304), 'conf_1.yml': (161.670422, 103.97521901130676)}
#

#p1 = subprocess.Popen(['time', 'mars_app'], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#(stdout, stderr) = p1.communicate()
#print("CPU Time: " + str(cpu_time))
#
#
#os.remove(CONF_FOLDER + "conf.yml")
#copyfile(CONF_FOLDER + "conf_2.yml", CONF_FOLDER+"conf.yml")
#time_output = StringIO.StringIO()
#subprocess.call(["time", "mars_app"])#, stdout=time_output)
#print(time_output)
#print timeit(stmt = "subprocess.call('mars_app')", setup = "import subprocess", number = 1)