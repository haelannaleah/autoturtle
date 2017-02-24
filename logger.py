""" ROS logging for ease of runtime documentation.
    
Author:
    Annaleah Ernst
"""
import csv
import rospy

from datetime import datetime
from time import time

class Logger:
    """ Create ROS style log messages.
    
    Args:
        name: The name of the module that owns the logger.
    """
    def __init__(self, name):
        self.__name__ = str(name)
        self._open_files = {}
        self._start_time = time()
    
    def _print(self, printer, msg):
        printer("[" + self.__name__ + "]: " + str(msg))
    
    def debug(self, msg, var_name = None, method = None, line = None):
        """ Log debugging messages.
            
        Currently set to ROS info for ease of access.
        
        Args:
            msg: The message. Generally a complete string or the contents of some variable.
            var_name (str, optional): Name of the variable whose contents are in msg.
            method (str, optional): Method from which the debug message originated.
            line (str/int, optional): Line number from which the debug message originated.
        """
        if var_name is not None:
            msg = str(var_name) + " = " + str(msg)
            
        if method is not None:
            msg = "In method '" + str(method) + "': " + str(msg)
            
        if line is not None:
            msg += " (line " + str(line) + ")"
            
        self._print(rospy.loginfo, msg)
        
    def error(self, msg, method = None):
        """ Log error messages. 
        
        Args:
            msg (str): Description of the error.
            method (str, optional): Method from which the error message originated.
        """
        if method is not None:
            msg = "Error in method '" + method + "': " + str(msg)
            
        self._print(rospy.logerr, msg)
    
    def info(self, msg):
        """ Log informative messages. 
        
        Args:
            msg (str): Informative message about program operation.
        """
        self._print(rospy.loginfo, msg)
    
    def warn(self, msg, method = None):
        """ Log warnings. 
        
        Args:
            msg (str): Description of the warning.
            method (str, optional): Method from which the warning message originated.
        """
        if method is not None:
            msg = "Warning in method '" + method + "': " + str(msg)
        
        self._print(rospy.logwarn, msg)
        
    def isLogging(self, tname):
        """ True if we've already started logging this test. """
        return tname in self._open_files

    def csv(self, tname, row, folder=None):
        """ Log data to a CSV file of the form filename_YYYYMMDD-HHMMSS.csv. 
        
        Args:
            tname (str): The name of the test. Note that this is not the same as the path to the file;
                rather, this should be descriptive of the test we are logging CSV data for.
            row (list): The line to be added to the CSV file.
            folder (str, optional): The name of the local file we want to store the file in.
        
        Note:
            When called on fname for the first time, Logger will assume that the row contains column 
                variable names for the following rows.
        """
        # if we haven't been writing to this already, open it up
        if tname not in self._open_files:
        
            # set filename to include current datetime and (optional) folder
            filename = self.__name__ + "_" + tname + datetime.now().strftime("_%Y%m%d-%H%M%S") + ".csv"
            if folder is not None:
                filename = folder + "/" + filename
            
            # open the file and set up the csv writer
            self._open_files[tname] = {}
            self._open_files[tname]["file"] = open(filename, "w+")
            self._open_files[tname]["writer"] = csv.writer(self._open_files[tname]["file"])
            
            # assume that the first message will be variable names
            self._open_files[tname]["writer"].writerow(["time"] + row)
    
        else:
            # preappend the current time and write current line to file
            self._open_files[tname]["writer"].writerow([time() - self._start_time] + row)

    def shutdown(self):
        """ Close any open logging files. """
        for tname in self._open_files:
            self._open_files[tname]["file"].close()

if __name__ == "__main__":
    from tester import Tester
    
    class LoggerTest(Tester):
        """ Unit tests for the Logger class. """
        
        def __init__(self):
            # initalize test class
            Tester.__init__(self, "Logger")
            
        def main(self):
            """ The main control loop. """
            
            # verify that log messages are functional
            self.logger.info("hello")
            self.logger.error("Error!")
            self.logger.error("Method error!", method="main")
            self.logger.debug("Debug!")
            self.logger.debug("Debug!", method = "main")
            self.logger.debug("Debug!", var_name = "myvar")
            self.logger.debug("Debug!", line = 91)
            self.logger.debug("Debug!", method = "main", line = "92")
            self.logger.debug("Debug!", var_name = "myvar", method = "main", line = 93)
            self.logger.warn("Warn!")
            self.logger.warn("Method warn!", method="main")
            
            self.logger.csv("test", ["hello", "world"])
            self.logger.csv("test", [1, 3])
            self.logger.csv("test2", ["hi", "again"])

            self.signal_shutdown("Logger test complete.")

    LoggerTest().run()