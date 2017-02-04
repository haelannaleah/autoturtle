""" ROS logging for ease of runtime documentation.
    
Author:
    Annaleah Ernst
"""

import rospy

class Logger:
    """ Create ROS style log messages.
    
    Args:
        name: The name of the module that owns the logger.
    """
    def __init__(self, name):
        self.__name__ = "[" + str(name) + "]: "
    
    def _print(self, printer, msg):
        printer(self.__name__ + str(msg))
    
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

            self.signal_shutdown("Logger test complete.")

    LoggerTest()