from unitree_api.msg import Request
import json

ROBOT_SPORT_API_ID_DAMP               = 1001
ROBOT_SPORT_API_ID_BALANCESTAND       = 1002
ROBOT_SPORT_API_ID_STOPMOVE           = 1003
ROBOT_SPORT_API_ID_STANDUP            = 1004
ROBOT_SPORT_API_ID_STANDDOWN          = 1005
ROBOT_SPORT_API_ID_RECOVERYSTAND      = 1006
ROBOT_SPORT_API_ID_EULER              = 1007
ROBOT_SPORT_API_ID_MOVE               = 1008
ROBOT_SPORT_API_ID_SIT                = 1009
ROBOT_SPORT_API_ID_RISESIT            = 1010
ROBOT_SPORT_API_ID_SWITCHGAIT         = 1011
ROBOT_SPORT_API_ID_TRIGGER            = 1012
ROBOT_SPORT_API_ID_BODYHEIGHT         = 1013
ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT    = 1014
ROBOT_SPORT_API_ID_SPEEDLEVEL         = 1015
ROBOT_SPORT_API_ID_HELLO              = 1016
ROBOT_SPORT_API_ID_STRETCH            = 1017
ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW   = 1018
ROBOT_SPORT_API_ID_CONTINUOUSGAIT     = 1019
ROBOT_SPORT_API_ID_CONTENT            = 1020
ROBOT_SPORT_API_ID_WALLOW             = 1021
ROBOT_SPORT_API_ID_DANCE1             = 1022
ROBOT_SPORT_API_ID_DANCE2             = 1023
ROBOT_SPORT_API_ID_GETBODYHEIGHT      = 1024
ROBOT_SPORT_API_ID_GETFOOTRAISEHEIGHT = 1025
ROBOT_SPORT_API_ID_GETSPEEDLEVEL      = 1026
ROBOT_SPORT_API_ID_SWITCHJOYSTICK     = 1027
ROBOT_SPORT_API_ID_POSE               = 1028
ROBOT_SPORT_API_ID_SCRAPE             = 1029
ROBOT_SPORT_API_ID_FRONTFLIP          = 1030
ROBOT_SPORT_API_ID_FRONTJUMP          = 1031
ROBOT_SPORT_API_ID_FRONTPOUNCE        = 1032
ROBOT_SPORT_API_ID_WIGGLEHIPS         = 1033
ROBOT_SPORT_API_ID_GETSTATE           = 1034
ROBOT_SPORT_API_ID_ECONOMICGAIT       = 1035
ROBOT_SPORT_API_ID_HEART              = 1036

class SportClient:

    @staticmethod
    def Damp(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_DAMP

    @staticmethod
    def BalanceStand(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND

    @staticmethod
    def StopMove(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE

    @staticmethod
    def StandUp(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDUP

    @staticmethod
    def StandDown(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDDOWN

    @staticmethod
    def RecoveryStand(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_RECOVERYSTAND

    @staticmethod
    def Euler(req: Request, roll: float, pitch: float, yaw: float):
        js = {}
        js["x"] = roll
        js["y"] = pitch
        js["z"] = yaw
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_EULER

    @staticmethod
    def Move(req: Request, vx: float, vy: float, vyaw: float):
        js = {}
        js["x"] = vx
        js["y"] = vy
        js["z"] = vyaw
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE

    @staticmethod
    def Sit(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_SIT

    @staticmethod
    def RiseSit(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_RISESIT

    @staticmethod
    def SwitchGait(req: Request, d: int):
        js = {}
        js["data"] = d
        req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHGAIT
        req.parameter = json.dumps(js)

    @staticmethod
    def Trigger(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_TRIGGER

    @staticmethod
    def BodyHeight(req: Request, height: float):
        js = {}
        js["data"] = height
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_BODYHEIGHT

    @staticmethod
    def FootRaiseHeight(req: Request, height: float):
        js = {}
        js["data"] = height
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT

    @staticmethod
    def SpeedLevel(req: Request, level: int):
        js = {}
        js["data"] = level
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_SPEEDLEVEL

    @staticmethod
    def Hello(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_HELLO

    @staticmethod
    def Stretch(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STRETCH

    # @staticmethod
    # def TrajectoryFollow(req: Request, std::vector<PathPoint> &path):
    #     nlohmann::json js_path
    #     req.header.identity.api_id = ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW
    #     for (int i = 0 i < 30 i++)
    #     {
    #         nlohmann::json js_point
    #         js_point["t_from_start"] = path[i].timeFromStart
    #         js_point["x"] = path[i].x
    #         js_point["y"] = path[i].y
    #         js_point["yaw"] = path[i].yaw
    #         js_point["vx"] = path[i].vx
    #         js_point["vy"] = path[i].vy
    #         js_point["vyaw"] = path[i].vyaw
    #         js_path.push_back(js_point)
    #     }
    #     req.parameter =js_path.dump()

    @staticmethod
    def SwitchJoystick(req: Request, flag: bool):
        js = {}
        js["data"] = flag
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHJOYSTICK

    @staticmethod
    def ContinuousGait(req: Request, flag: bool):
        js = {}
        js["data"] = flag
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTINUOUSGAIT

    @staticmethod
    def Wallow(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_WALLOW

    @staticmethod
    def Content(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTENT

    @staticmethod
    def Pose(req: Request, flag: bool):
        js = {}
        js["data"] = flag
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_POSE

    @staticmethod
    def Scrape(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_SCRAPE

    @staticmethod
    def FrontFlip(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTFLIP

    @staticmethod
    def FrontJump(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTJUMP

    @staticmethod
    def FrontPounce(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTPOUNCE

    @staticmethod
    def Dance1(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE1

    @staticmethod
    def Dance2(req: Request):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE2
