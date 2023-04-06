# ROS specific imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from rcl_interfaces.msg import ParameterType, Parameter
from task_manager_msgs.msg import *
from task_manager_msgs.srv import *
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# python3
from functools import reduce
import argparse
import threading


class TaskException(Exception):
    def __init__(self, value,id=None,status=None,statusString=""):
        self.id = id
        self.value = value
        self.status = status
        self.statusString = statusString
    def __str__(self):
        return repr(self.value)

class TaskConditionException(Exception):
    def __init__(self, value, conds):
        self.value = value
        self.conditions = conds
    def __str__(self):
        return repr(self.value)

class Condition:
    def __init__(self,name):
        self.name = name
        test = self.isVerified()

    def __str__(self):
        return self.name

class NegatedCondition(Condition):
    def __init__(self,cond):
        self.name = "not " + cond.name
        self.cond = cond
    def isVerified(self):
        return not self.cond.isVerified()

class ConditionIsCompleted(Condition):
    def __init__(self, name, tc, taskId):
        self.name = name
        self.tc = tc
        self.taskId = taskId

    def isVerified(self):
        if not self.tc.isKnown(self.taskId):
            return False
        return self.tc.isCompleted(self.taskId)

class ConditionVariable(Condition):
    def __init__(self, name):
        self.name = name
        self.var = False

    def set(self,val):
        self.var = val

    def isVerified(self):
        return self.var

class ConditionIsRunning(Condition):
    def __init__(self, name, tc, taskId):
        self.name = name
        self.tc = tc
        self.taskId = taskId

    def isVerified(self):
        if not self.tc.isKnown(self.taskId):
            return False
        return not self.tc.isCompleted(self.taskId)

class TaskClient(Node):
    sock = None
    verbose = 0
    messageid = 0
    keepAlive = False
    tasklist = {}
    taskstatus = {}
    conditions = []
    status_functions = []

    taskStatusStrings = dict([( TaskStatus.__dict__[k],k) for k in TaskStatus.__dict__.keys() if k[0:5]=="TASK_"])
    taskStatusId = dict([(v,k) for k,v in taskStatusStrings.items()])

    def addCondition(self,cond):
        self.conditions.append(cond)

    def clearConditions(self):
        self.conditions = []

    def anyConditionVerified(self):
        return reduce(lambda x,y:x or y,[x.isVerified() for x in self.conditions],False)

    def allConditionsVerified(self):
        return reduce(lambda x,y:x and y,[x.isVerified() for x in self.conditions],False)

    def getVerifiedConditions(self):
        v = []
        for x in self.conditions:
            if x.isVerified():
                v.append(x)
        return v

    def registerStatusFunction(self,f):
        self.status_functions.append(f)

    class TaskDefinition:
        name = ""
        help = ""
        client = None
        def __init__(self,name,help,periodic,cfg,client):
            self.name = name
            self.help = help
            self.periodic = periodic
            self.config = cfg
            self.params = dict([(p.name,p) for p in cfg])

            self.conv = dict()
            for p in self.params.values():
                if p.type == ParameterType.PARAMETER_INTEGER:
                    self.conv[p.name] = int
                if p.type == ParameterType.PARAMETER_DOUBLE:
                    self.conv[p.name] = float
                if p.type == ParameterType.PARAMETER_STRING:
                    self.conv[p.name] = str
                if p.type == ParameterType.PARAMETER_BOOL:
                    self.conv[p.name] = bool

            # print("DEBUG:", cfg)
            # params = extract_params(decode_description(self.config))
            self.client = client

        def prepareParams(self,paramdict):
            for p in paramdict:
                if p not in self.conv:
                    raise NameError("%s: Parameter '%s' is not declared for task '%s'" % (self.client.server_node,p,self.name))
                try:
                    paramdict[p] = self.conv[p](paramdict[p])
                except ValueError:
                    raise ValueError("%s: Could not convert argument '%s' from '%s' to '%s'"
                            % (self.client.server_node,p, str(paramdict[p]), self.params[p].type))

            paramdict['task_name'] = self.name
            return paramdict

        def start(self,**paramdict):
            argv = None
            if ('argv' in paramdict):
                argv = paramdict['argv']
                del paramdict['argv']
            paramdict = self.prepareParams(paramdict)
            id = self.client.startTask(paramdict)
            return id

        def __call__(self,**paramdict):
            argx = None
            if ('argv' in paramdict):
                argx = paramdict['argv']
                del paramdict['argv']
            paramdict = self.prepareParams(paramdict)
            foreground = True
            if ('foreground' in paramdict):
                foreground = bool(paramdict['foreground'])
            if (foreground):
                self.client.get_logger().info("%s: Starting task %s in foreground" % (self.client.server_node,self.name))
                res = self.client.startTaskAndWait(paramdict,argv=argx)
                return res
            else:
                id = self.client.startTask(paramdict,argv=argx)
                self.get_logger().info("%s: Starting task %s in background: %d" % (self.client.server_node,self.name,id))
                return id

    class TaskStatus:
        def __init__(self,client):
            self.client = client
            self.id = 0
            self.name = ""
            self.status = 0
            self.foreground = True
            self.statusString = ""
            self.statusTime = 0.0

        def __str__(self):
            output = "%f %-12s " % (self.statusTime,self.name)
            if (self.foreground):
                output += "F "
            else:
                output += "B "
            output += self.client.status_string(self.status)
            return output

    def __init__(self,server_node,default_period):
        super().__init__('task_client')
        self.serviceLock = threading.RLock()
        self.statusLock = threading.RLock()
        self.statusCond = threading.Condition(self.statusLock)
        # self.status_cb_group = ReentrantCallbackGroup()
        parser = argparse.ArgumentParser(description='Client to run and control tasks on a given server node')
        parser.add_argument('--server', '-s',default=server_node,required=(server_node==""),
                nargs=1, help='server node name, e.g. /task_server', type=str)
        parser.add_argument('--period', '-p',default=default_period,type=float,
                nargs=1, help='default period for new tasks')
        parser.add_argument('--check', '-c',action='store_const', const=True, dest='check', default=False,
                help='if set, only test task syntax, but do not run')
        args,unknown = parser.parse_known_args()
        # print args
        self.default_period=args.period
        if type(args.server) is list:
            self.server_node=args.server[0]
        else:
            self.server_node=args.server
        self.check_only=args.check

        self.get_logger().info("Creating link to services on node " + self.server_node)
        if self.check_only:
            self.get_logger().info("%s: Dry-run only: this might not work for complex mission"%self.server_node)

        self.cli_get_all_tasks = self.create_client(GetTaskList, self.server_node + '/get_all_tasks')
        self.cli_start_task = self.create_client(StartTask, self.server_node + '/start_task')
        self.cli_stop_task = self.create_client(StopTask, self.server_node + '/stop_task')
        self.cli_get_all_status = self.create_client(GetAllTaskStatus, self.server_node + '/get_all_status')

        while not self.cli_get_all_tasks.wait_for_service(1.0):
            self.get_logger().info('Service ' + self.server_node + '/get_all_tasks not available...')
        while not self.cli_start_task.wait_for_service(1.0):
            self.get_logger().info('Service ' + self.server_node + '/start_task not available...')
        while not self.cli_stop_task.wait_for_service(1.0):
            self.get_logger().info('Service ' + self.server_node + '/stop_task not available...')
        while not self.cli_get_all_status.wait_for_service(1.0):
            self.get_logger().info('Service ' + self.server_node + '/get_all_status not available...')

        self.req_get_all_tasks = GetTaskList.Request()
        self.req_start_task = StartTask.Request()
        self.req_stop_task = StopTask.Request()
        self.req_get_all_status = GetAllTaskStatus.Request()

        self.keepAlivePub = self.create_publisher(Header, self.server_node + '/keep_alive', 1)
        self.statusSub = self.create_subscription(TaskStatus, self.server_node + "/status", self.status_callback, 50)
        self.timer = self.create_timer(0.1, self.timerCallback)


        self.updateTaskList()
        self.updateTaskStatus()

        self.nothing_new = True
        self.idle()

    def __del__(self):
        # print("DEBUG: Deleting task client object")
        try:
            if rclpy.ok():
                self.idle()
        except Exception as e:
            pass

    def __getattr__(self,name):
        if name=="__dir__":
            return self.tasklist.keys
        return self.tasklist[name]

    def timerCallback(self):
        if self.keepAlive and rclpy.ok():
            try:
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                self.keepAlivePub.publish(header)
            except Exception as e:
                print(e)
                # Ignore, this sometimes happens on shutdown
                pass

    def updateTaskList(self):
        # print("DEBUG: update the task list")
        # try:
        with self.serviceLock:
            self.future = self.cli_get_all_tasks.call_async(self.req_get_all_tasks)
            rclpy.spin_until_future_complete(self, self.future)
            resp = self.future.result()
        self.tasklist = {}
        for t in resp.tlist:
            # print(t.name)
            self.tasklist[t.name] = self.TaskDefinition(t.name,t.description,t.periodic,t.config,self)
        # except Exception as e:
            # self.get_logger().error("Service call failed: %s"%e)

    def printTaskList(self):
        for k,v in self.tasklist.items():
            print("Task %s: %s" % (k,v.help))

    def encodeConfig(self, paramdict):
        config = []
        for p in paramdict:
            P = Parameter()
            P.name = p
            value = paramdict[p]
            if type(value)==int:
                P.value.type = ParameterType.PARAMETER_INTEGER
                P.value.integer_value = value
            elif type(value)==float:
                P.value.type = ParameterType.PARAMETER_DOUBLE
                P.value.double_value = value
            elif type(value)==bool:
                P.value.type = ParameterType.PARAMETER_BOOL
                P.value.bool_value = value
            elif type(value)==str:
                P.value.type = ParameterType.PARAMETER_STRING
                P.value.string_value = value
            config.append(P)
        return config

    def startTask(self,paramdict,name="",foreground=True,period=-1,argv=None):
        if not rclpy.ok():
            raise TaskException("Aborting due to ROS shutdown")
        if self.check_only:
            self.sleep()
            return 0
        if period < 0:
            period = self.default_period
        try:
            if ('task_name' in paramdict):
                name=paramdict['task_name']
                del paramdict['task_name']
            if ('foreground' not in paramdict):
                paramdict['foreground'] = bool(foreground)
            else:
                paramdict['foreground'] = bool(paramdict['foreground'])
            if (period>0) and ('task_period' not in paramdict):
                paramdict['task_period'] = float(period)
            config = self.encodeConfig(paramdict)
            if argv:
                extra = EncapsulatedMessage()
                self.get_logger().error("Not implemented")
            else:
                extra = EncapsulatedMessage()
            # print config
            self.get_logger().info("Starting task %s" % name)
            with self.serviceLock:
                self.req_start_task.name = name
                self.req_start_task.config = config
                self.future = self.cli_start_task.call_async(self.req_start_task)
                rclpy.spin_until_future_complete(self, self.future)
                resp = self.future.result()
            self.keepAlive = True
            return resp.id
        except Exception as e:
            self.get_logger().error("Service call failed: %s"%e)
            raise

    def startTaskAndWait(self,paramdict,name="",foreground=True,period=-1.,argv=None):
        tid = self.startTask(paramdict,name,foreground,period,argv)
        if (self.verbose):
            self.get_logger().info("Waiting task %d" % tid)
        if self.check_only:
            return True
        return self.waitTask(tid)

    def stopTask(self,id):
        try:
            with self.serviceLock:
                self.req_stop_task.id = id
                self.future = self.cli_stop_task.call_async(self.req_stop_task)
                rclpy.spin_until_future_complete(self, self.future)
                resp = self.future.result()
            return 0
        except Exception as e:
            self.get_logger().error( "Service call failed: %s"%e)
            raise

    def idle(self):
        try:
            with self.serviceLock:
                self.req_stop_task.id = -1
                self.future = self.cli_stop_task.call_async(self.req_stop_task)
                rclpy.spin_until_future_complete(self, self.future)
                resp = self.future.result()
            return 0
        except Exception as e:
            self.get_logger().error( "Service call failed: %s"%e)
            raise

    def status_string(self,v):
        statusTerminated = self.taskStatusId["TASK_TERMINATED"]
        if (v & statusTerminated):
            return self.taskStatusStrings[v & ~statusTerminated] + " & TASK_TERMINATED"
        else:
            return self.taskStatusStrings[v]

    def status_callback(self,t):
        # self.get_logger().info("status cb")
        # t0 = self.get_clock().now().nanoseconds / 1e9
        with self.statusLock:
            ts = self.TaskStatus(self)
            ts.id = t.id
            ts.name = t.name
            status = t.status
            # print((ts.id,ts.name,status,t.status_time,self.get_clock().now()))
            ts.status = status & 0x000000FF
            ts.foreground = bool(status & 0x100)
            ts.statusString = t.status_string
            ts.statusTime = Time.from_msg(t.status_time).nanoseconds / 1e9
            self.taskstatus[ts.id] = ts
            t = self.get_clock().now().nanoseconds / 1e9
            # self.get_logger().info(f"STATUS CB after lock: {t - t0}")
            # self.get_logger().info(f"STATUS CB id: {ts.id} name: {ts.name} time_diff: {t - ts.statusTime} status:{ts.status} string:{ts.statusString}")
            to_be_deleted = []
            for k,v in self.taskstatus.items():
                if (t - v.statusTime) > 10.0:
                    to_be_deleted.append(k)
            for k in to_be_deleted:
                del self.taskstatus[k]
            for f in self.status_functions:
                f(ts)
        try:
            self.statusCond.notify_all()
        except RuntimeError:
            # Nobody is waiting
            pass
        self.nothing_new = False

    def isKnown(self,taskId):
        with self.statusLock:
            return taskId in self.taskstatus.keys()

    def isCompleted(self,taskId,requireKnown=True):
        with self.statusLock:
            if requireKnown and not self.isKnown(taskId):
                return False
            if not self.isKnown(taskId):
                return True
            return self.taskstatus[taskId].status & self.taskStatusId['TASK_TERMINATED']

    def updateTaskStatus(self):
        try:
            with self.statusLock:
                with self.serviceLock:
                    # resp = self.get_status()
                    self.future = self.cli_get_all_status.call_async(self.req_get_all_status)
                    rclpy.spin_until_future_complete(self, self.future)
                    resp = self.future.result()
                for t in resp.running_tasks + resp.zombie_tasks:
                    ts = self.TaskStatus(self)
                    ts.id = t.id
                    ts.name = t.name
                    status = t.status
                    ts.status = status & 0x000000FF
                    ts.foreground = bool(status & 0x100)
                    ts.statusString = t.status_string
                    ts.statusTime = Time.from_msg(t.status_time).nanoseconds / 1e9
                    self.taskstatus[ts.id] = ts
            try:
                self.statusCond.notify_all()
            except RuntimeError:
                # Nobody is waiting
                pass
        except Exception as e:
            self.get_logger().error( "%s: Service call failed: %s"%(self.server_node,e))
            raise

    def waitTaskList(self,ids,wait_for_all, stop_others):
        statusTerminated = self.taskStatusId['TASK_TERMINATED']
        t0 = self.get_clock().now()
        completed = dict([(id,False) for id in ids])
        red_fun = lambda x,y: x or y
        if wait_for_all:
            red_fun = lambda x,y: x and y

        with self.statusLock:
            while rclpy.ok():
                self.nothing_new=True
                while rclpy.ok() and self.nothing_new:
                    rclpy.spin_once(self,timeout_sec=0.1)
                # This would be a good idea, except that python is not
                # receiving the messages while waiting for the condition
                # self.statusCond.wait(0.020)
                if self.verbose>1:
                    self.printTaskStatus()
                t1 = self.get_clock().now()
                if (self.anyConditionVerified()):
                    for id in ids:
                        self.stopTask(id)
                    trueConditions = self.getVerifiedConditions();
                    self.clearConditions()
                    self.get_logger().info("%s: Task %s terminated on condition" % (self.server_node,str(ids)))
                    raise TaskConditionException("%s: Task %s terminated on condition" % (self.server_node,str(ids)),trueConditions)
                for id in ids:
                    if id not in self.taskstatus:
                        if (t1-t0) > rclpy.duration.Duration(seconds=2.0): 
                            if (self.verbose):
                                self.get_logger().error("%s: Id %d not in taskstatus" % (self.server_node,id))
                            raise TaskException("%s: Task %d did not appear in task status" % (self.server_node,id),id);
                    else:
                        if self.verbose>1:
                            print("%s: %d: %02X - %s\n%s" % (self.server_node,id, self.taskstatus[id].status,self.status_string(self.taskstatus[id].status),self.taskstatus[id].statusString))
                        if not (self.taskstatus[id].status & statusTerminated):
                            continue
                        status = self.taskstatus[id].status & (~statusTerminated)
                        if (status == self.taskStatusId["TASK_COMPLETED"]):
                            if (self.verbose):
                                self.get_logger().info("%s: Task %d terminated (%d)" % (self.server_node,id,status))
                            completed[id] = True
                        elif (status > self.taskStatusId["TASK_COMPLETED"]):
                            if (self.verbose):
                                self.get_logger().warn( "%s: Task %d failed (%d - %s)" %  (self.server_node,id,status,self.status_string(status)))
                            raise TaskException("%s: Task %d:%s failed: %d:%s" % (self.server_node,id,self.taskstatus[id].name,status,self.status_string(status)), id, status,self.taskstatus[id].statusString);
                            # instead of raise?
                            # completed[id] = True
                if reduce(red_fun,completed.values()):
                    if stop_others:
                        for k,v in completed.items():
                            if not v:
                                self.stopTask(k)
                    return True
            if not rclpy.ok():
                raise TaskException("%s: Aborting due to ROS shutdown"%self.server_node);
            return False

    def stopAllTasks(self):
        self.keepAlive = False
        statusTerminated = self.taskStatusId['TASK_TERMINATED']
        for id in self.taskstatus:
            if self.taskstatus[id].status & (~statusTerminated):
                continue
            self.stopTask(id)

    def waitTask(self,id):
        return self.waitTaskList([id],True,False)

    def waitAnyTasks(self,ids,stop_others=True):
        return self.waitTaskList(ids,False,stop_others)

    def waitAllTasks(self,ids):
        return self.waitTaskList(ids,True,False)

    def printTaskStatus(self):
        with self.statusLock:
            for k,v in self.taskstatus.items():
                print("Task %d: %s" % (k,str(v)))










