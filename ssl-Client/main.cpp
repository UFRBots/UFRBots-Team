//author  Renato Sousa, 2018
#include <QtNetwork>
#include <stdio.h>
#include <QtCore>
#include <QUdpSocket>
#include <iostream>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"

// referee placement packets
#include "include/vssref_placement.pb.h"
#include "include/vssref_command.pb.h"

#include "net/pb/command.pb.h"
#include "net/pb/common.pb.h"
#include "net/pb/packet.pb.h"
#include "net/pb/replacement.pb.h"

enum 
{
    APROXIMA = 0,
    DECIDE_DESVIO,
    SOBE,
    DESCE,
    VOLTA
};

class Objective 
{
    double m_x;
    double m_y;
    double m_angle;

    public:
        Objective(double t_x, double t_y, double t_angle) : m_x(t_x), m_y(t_y), m_angle(t_angle){};

        void setY(double value)
        {
            m_y = value;
        }

        void setAngle(double value)
        {
            m_angle = value;
        }

        void setX(double value)
        {
            m_x = value;
        }
        double x()
        {
            return m_x;
        }
        double y()
        {
            return m_y;
        }
        double angle()
        {
            return m_angle;
        }
};

//void printRobotInfo(const fira_message::Robot &robot)
//{
//    printf("ID=%3d \n", robot.robot_id());

//    printf(" POS=<%9.2f,%9.2f> \n", robot.x(), robot.y());
//    printf(" VEL=<%9.2f,%9.2f> \n", robot.vx(), robot.vy());

//    printf("ANGLE=%6.3f \n", robot.orientation());
//    printf("ANGLE VEL=%6.3f \n", robot.vorientation());
//}

double to180range(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle < -M_PI)
    {
        angle = angle + 2 * M_PI;
    }
    else if (angle > M_PI)
    {
        angle = angle - 2 * M_PI;
    }
    return angle;
}

double smallestAngleDiff(double target, double source)
{
    double a;
    a = fmod(target + 2 * M_PI, 2 * M_PI) - fmod(source + 2 * M_PI, 2 * M_PI);

    if (a > M_PI)
    {
        a = a - 2 * M_PI;
    }
    else if (a < -M_PI)
    {
        a = a + 2 * M_PI;
    }
    return a;
}

void PID(fira_message::Robot robot, Objective objective, int index, GrSim_Client &commandClient, bool Team_UFRBots, double acrescimo)
{
    double Kp = 20;
    double Kd = 2.5;
    static double lastError = 0;

    double rightMotorSpeed;
    double leftMotorSpeed;

    bool reversed = false;

    double angle_rob = robot.orientation();

    double angle_obj = atan2(objective.y() - robot.y(), objective.x() - robot.x());

    double error = smallestAngleDiff(angle_rob, angle_obj);

    if (fabs(error) > M_PI / 2.0 + M_PI / 20.0)
    {
        reversed = true;
        angle_rob = to180range(angle_rob + M_PI);
        // Calculates the error and reverses the front of the robot
        error = smallestAngleDiff(angle_rob, angle_obj);
    }

    double motorSpeed = (Kp * error) + (Kd * (error - lastError)); // + 0.2 * sumErr;
    lastError = error;

    double baseSpeed = (30 + acrescimo);

    // Normalize
    motorSpeed = motorSpeed > (30 + acrescimo) ? (30 + acrescimo) : motorSpeed;
    motorSpeed = motorSpeed < (-30 - acrescimo) ? (-30 - acrescimo)  : motorSpeed;

    if (motorSpeed > 0)
    {
        leftMotorSpeed = baseSpeed;
        rightMotorSpeed = baseSpeed - motorSpeed;
    }
    else
    {
        leftMotorSpeed = baseSpeed + motorSpeed;
        rightMotorSpeed = baseSpeed;
    }

    if (reversed)
    {
        if (motorSpeed > 0)
        {
            leftMotorSpeed = -baseSpeed + motorSpeed;
            rightMotorSpeed = -baseSpeed;
        }
        else
        {
            leftMotorSpeed = -baseSpeed;
            rightMotorSpeed = -baseSpeed - motorSpeed;
        }
    }
    commandClient.sendCommand(leftMotorSpeed, rightMotorSpeed, Team_UFRBots, index);
}

int estado = APROXIMA;
int estadoant;
double x, y, ang;
double x_in, y_in;

Objective defineObjective(fira_message::Robot robot, fira_message::Ball ball)
{
    double distancia = sqrt(pow(robot.x() - ball.x(), 2) + pow(robot.y() - ball.y(), 2));

    if (robot.x() < ball.x())
    { //se o robô está atrás da bola
        estado = APROXIMA;

        if ((robot.y() < ball.y() - 5 || robot.y() > ball.y() + 5) && robot.x() <= 22)
            //alinhando o robô com a bola!
            return Objective(ball.x() - 10, ball.y(), M_PI / 4.); // x,y,angle

        else if (distancia < 8 && 49 < robot.y() && 83 > robot.y())
        {
            //Se o robô 'está' com a bola e está indo em direção ao gol
            return Objective(ball.x(), ball.y(), M_PI / 4.);
        }
        else if (distancia < 8 && (43 > robot.y() || 83 < robot.y()))
        {
            //Se o robô 'está' com a bola e NÃO está indo em direção ao gol
            return Objective(ball.x(), 66, M_PI / 4.);
        }
        else
            return Objective(ball.x(), ball.y(), M_PI / 4.);
        //no mais, corre atrás da bola...
    }
    else
    {
        switch (estado)
        {
        
        case APROXIMA:
            //corre atras da bola ate chegar perto
            if (distancia < 10)
                estado = DECIDE_DESVIO;
            x = ball.x() + 10;
            y = ball.y();
            ang = 0; // x,y,angle
            break;

        case DECIDE_DESVIO:
            //desvia da bola, para voltar a ficar atrás dela
            if (robot.y() - 10 > 6)
                estado = SOBE;
            else
                estado = DESCE;

            break;

        case SOBE:
            estadoant = SOBE;
            estado = VOLTA;
            x = ball.x() + 10;
            y = ball.y() - 8;
            x_in = robot.x();
            y_in = robot.y();
            ang = M_PI / 2.0; // x,y,angle

            break;
        
        case DESCE:
            estadoant = DESCE;
            estado = VOLTA;
            x = ball.x() + 10;
            y = ball.y() + 8;
            x_in = robot.x();
            y_in = robot.y();
            ang = M_PI / 2.0; // x,y,angle

            break;
        
        case VOLTA:
            if (robot.x() <= x_in - 16 || robot.x() <= 12)
            {
                //se ele andou 16 ou tá no limite do mapa
                //fazendo_manobra = 0;
                estado = APROXIMA;
            }
            if (estadoant == DESCE)
            {
                x = ball.x() - 16;
                y = ball.y() + 8;
                ang = M_PI; // x,y,angle */
            }
            else
            {
                x = ball.x() - 16;
                y = ball.y() - 8;
                ang = M_PI; // x,y,angle */
            }
            break;
        }

        return Objective(x, y, ang);
    }
}


Objective defineObjectiveYellow(fira_message::Robot robot, fira_message::Ball ball)
{
    double distancia = sqrt(pow(robot.x() - ball.x(), 2) + pow(robot.y() - ball.y(), 2));

    if (robot.x() > ball.x())
    { //se o robô está atrás da bola
        estado = APROXIMA;

        if ((robot.y() < ball.y() - 5 || robot.y() > ball.y() + 5) && robot.x() >= 148)
            //alinhando o robô com a bola!
            return Objective(ball.x() - 10, ball.y(), M_PI / 4.); // x,y,angle

        else if (distancia < 8 && 49 < robot.y() && 83 > robot.y())
        {
            //Se o robô 'está' com a bola e está indo em direção ao gol
            return Objective(ball.x(), ball.y(), M_PI / 4.);
        }
        else if (distancia < 8 && (43 > robot.y() || 83 < robot.y()))
        {
            //Se o robô 'está' com a bola e NÃO está indo em direção ao gol
            return Objective(ball.x(), 66, M_PI / 4.);
        }
        else
            return Objective(ball.x(), ball.y(), M_PI / 4.);
        //no mais, corre atrás da bola...
    }
    else
    {
        switch (estado)
        {

        case APROXIMA:
            //corre atras da bola ate chegar perto
            if (distancia < 10)
                estado = DECIDE_DESVIO;
            x = ball.x() - 10;
            y = ball.y();
            ang = 0; // x,y,angle
            break;

        case DECIDE_DESVIO:
            //desvia da bola, para voltar a ficar atrás dela
            if (robot.y() - 10 > 6)
                estado = SOBE;
            else
                estado = DESCE;
            break;

        case SOBE:
            estadoant = SOBE;
            estado = VOLTA;
            x = ball.x() - 10;
            y = ball.y() - 8;
            x_in = robot.x();
            y_in = robot.y();
            ang = M_PI / 2.0; // x,y,angle
            break;

        case DESCE:
            estadoant = DESCE;
            estado = VOLTA;
            x = ball.x() - 10;
            y = ball.y() + 8;
            x_in = robot.x();
            y_in = robot.y();
            ang = M_PI / 2.0; // x,y,angle

            break;

        case VOLTA:
            if (robot.x() <= x_in + 16 || robot.x() >= 168)
            {
                //se ele andou 16 ou tá no limite do mapa
                //fazendo_manobra = 0;
                estado = APROXIMA;
            }
            if (estadoant == DESCE)
            {
                x = ball.x() + 16;
                y = ball.y() + 8;
                ang = M_PI; // x,y,angle */
            }
            else
            {
                x = ball.x() + 16;
                y = ball.y() - 8;
                ang = M_PI; // x,y,angle */
            }
            break;
        }

        return Objective(x, y, ang);
    }
}



QString getFoulNameById(VSSRef::Foul foul){
    switch(foul){
        case VSSRef::Foul::FREE_BALL:    return "FREE_BALL";
        case VSSRef::Foul::FREE_KICK:    return "FREE_KICK";
        case VSSRef::Foul::GOAL_KICK:    return "GOAL_KICK";
        case VSSRef::Foul::PENALTY_KICK: return "PENALTY_KICK";
        case VSSRef::Foul::KICKOFF:      return "KICKOFF";
        case VSSRef::Foul::STOP:         return "STOP";
        case VSSRef::Foul::GAME_ON:      return "GAME_ON";
        default:                         return "FOUL NOT IDENTIFIED";
    }
}

QString getTeamColorNameById(VSSRef::Color color){
    switch(color){
        case VSSRef::Color::NONE:    return "NONE";
        case VSSRef::Color::BLUE:    return "BLUE";
        case VSSRef::Color::YELLOW:  return "YELLOW";
        default:                     return "COLOR NOT IDENTIFIED";
    }
}

QString getQuadrantNameById(VSSRef::Quadrant quadrant){
    switch(quadrant){
        case VSSRef::Quadrant::NO_QUADRANT: return "NO QUADRANT";
        case VSSRef::Quadrant::QUADRANT_1:  return "QUADRANT 1";
        case VSSRef::Quadrant::QUADRANT_2:  return "QUADRANT 2";
        case VSSRef::Quadrant::QUADRANT_3:  return "QUADRANT 3";
        case VSSRef::Quadrant::QUADRANT_4:  return "QUADRANT 4";
        default:                            return "QUADRANT NOT IDENTIFIED";
    }
}

QString getHalfNameById(VSSRef::Half half){
    switch(half){
        case VSSRef::Half::NO_HALF: return "NO_HALF";
        case VSSRef::Half::FIRST_HALF: return "FIRST HALF";
        case VSSRef::Half::SECOND_HALF: return "SECOND HALF";
        default: return "NO HALF DEFINED";
    }
}



int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    std::string BLUE = "BLUE";
    std::string YELLOW = "YELLOW";

    std::string color = argv[1]; //YELLOW or BLUE
    std::string multicast_ip = argv[2]; // 224.0.0.1
    QString multicast_ip_q = argv[2];// 224.0.0.1
    QString command_ip = argv[3]; //127.0.0.1
    int command_port = atoi(argv[4]); //20011
    int vision_port = atoi(argv[5]); //10002
    int referee_port = atoi(argv[6]); //10003
    int replacer_port = atoi(argv[7]); //10004

    //Docker
    //sudo sh ufrbots.sh color multicast_ip command_ip command_port vision_port referee_port replacer_port
    //sudo sh ufrbots.sh BLUE 224.5.23.2 127.0.0.1 20011 10002 10003 10004

    //Local
    // ./main BLUE 224.5.23.2 127.0.0.1 20011 10002 10003 10004

    bool Team_UFRBots;

    if (color.compare(BLUE) == 0)
    {
        //    Time azul
        Team_UFRBots = false;
    }
    if (color.compare(YELLOW) == 0)
    {
        //    Time amarelo
        Team_UFRBots = true;
    }

    // IP do simulador
    RoboCupSSLClient visionClient(multicast_ip, vision_port);
    visionClient.open(false);

    GrSim_Client commandClient(command_ip, command_port);

    // Pacotes de mensagens
    fira_message::sim_to_ref::Environment packet;


    // QUdpSocket
    QUdpSocket *replacerSocket = new QUdpSocket();
    QUdpSocket *refereeClient = new QUdpSocket();

    // Performing connection to send Replacer commands
   if(replacerSocket->isOpen())
       replacerSocket->close();

    replacerSocket->connectToHost(multicast_ip_q, replacer_port, QIODevice::WriteOnly, QAbstractSocket::IPv4Protocol);
       std::cout << "[Example] Connected to REPLACER socket in port " << replacer_port << " and address = " << multicast_ip << ".\n";

    // Bindando
    if(refereeClient->bind(QHostAddress::AnyIPv4, referee_port, QUdpSocket::ShareAddress) == false){
        std::cout << "Failed to bind" << std::endl;
        exit(-1);
    }

    // Conectando ao grupo multicast
    if(refereeClient->joinMulticastGroup(QHostAddress(multicast_ip_q)) == false){
        std::cout << "Failed to join" << std::endl;
        exit(-1);
    }

    bool game_on = false;
    bool penalty = false;

    while (true)
    {
        if (visionClient.receive(packet))
        {

            while(refereeClient->hasPendingDatagrams()){
                VSSRef::ref_to_team::VSSRef_Command command;
                char *buffer = new char[65535];
                long long int packetLength = 0;

                packetLength = refereeClient->readDatagram(buffer, 65535);
                if(command.ParseFromArray(buffer, int(packetLength)) == false){
                    std::cout << "Error in parse" << std::endl;
                    exit(-1);
                }

                string quadrante = getQuadrantNameById(command.foulquadrant()).toStdString();
                string tipo_falta = getFoulNameById(command.foul()).toStdString();
                string time_falta = getTeamColorNameById(command.teamcolor()).toStdString();

                // If received command, let's debug it
                std::cout << "[Example] Succesfully received an command from ref: " << tipo_falta << " for team " << time_falta << std::endl;
                std::cout << quadrante << std::endl;

                if (getFoulNameById(command.foul()).toStdString() == "GAME_ON")
                {
                    game_on = true;
                }
                else{
                    game_on = false;
                }

                if(!Team_UFRBots && quadrante == "QUADRANT 3")
                {
                    // First creating an placement command for the blue team
                    VSSRef::team_to_ref::VSSRef_Placement placementCommandBlue;
                    VSSRef::Frame *placementFrameBlue = new VSSRef::Frame();
                    placementFrameBlue->set_teamcolor(VSSRef::Color::BLUE);

                    for(int x = 0; x < 3; x++){
                        VSSRef::Robot *robot = placementFrameBlue->add_robots();
                        robot->set_robot_id(x);
                        if (x==0)
                        {
                            robot->set_x(-0.7);
                            robot->set_y(-0.15);
                            robot->set_orientation(0.0);
                        }
                        if (x==1)
                        {
                            robot->set_x(-0.3);
                            robot->set_y(0.1);
                            robot->set_orientation(0.0);
                        }
                        if (x==2)
                        {
                            robot->set_x(-0.6);
                            robot->set_y(-0.4);
                            robot->set_orientation(0.0);
                        }

                    }

                    placementCommandBlue.set_allocated_world(placementFrameBlue);

                    // Sending blue
                    std::string msgBlue;
                    placementCommandBlue.SerializeToString(&msgBlue);
                    if(replacerSocket->write(msgBlue.c_str(), msgBlue.length()) == -1){
                        std::cout << "[Example] Failed to write to replacer socket: " << replacerSocket->errorString().toStdString() << std::endl;
                    }

                }

                if(!Team_UFRBots && quadrante == "QUADRANT 4")
                {
                    // First creating an placement command for the blue team
                    VSSRef::team_to_ref::VSSRef_Placement placementCommandBlue;
                    VSSRef::Frame *placementFrameBlue = new VSSRef::Frame();
                    placementFrameBlue->set_teamcolor(VSSRef::Color::BLUE);

                    for(int x = 0; x < 3; x++){
                        VSSRef::Robot *robot = placementFrameBlue->add_robots();
                        robot->set_robot_id(x);
                        if (x==0)
                        {
                            robot->set_x(-0.7);
                            robot->set_y(-0.15);
                            robot->set_orientation(0.0);
                        }
                        if (x==1)
                        {
                            robot->set_x(0.1);
                            robot->set_y(0.23);
                            robot->set_orientation(0.0);
                        }
                        if (x==2)
                        {
                            robot->set_x(0.18);
                            robot->set_y(-0.4);
                            robot->set_orientation(0.0);
                        }

                    }

                    placementCommandBlue.set_allocated_world(placementFrameBlue);

                    // Sending blue
                    std::string msgBlue;
                    placementCommandBlue.SerializeToString(&msgBlue);
                    if(replacerSocket->write(msgBlue.c_str(), msgBlue.length()) == -1){
                        std::cout << "[Example] Failed to write to replacer socket: " << replacerSocket->errorString().toStdString() << std::endl;
                    }

                }

                if(!Team_UFRBots && tipo_falta == "PENALTY_KICK" && time_falta == "BLUE")
                {
                    // First creating an placement command for the blue team
                    VSSRef::team_to_ref::VSSRef_Placement placementCommandBlue;
                    VSSRef::Frame *placementFrameBlue = new VSSRef::Frame();
                    placementFrameBlue->set_teamcolor(VSSRef::Color::BLUE);

                    penalty = true;

                    for(int x = 0; x < 3; x++){
                        VSSRef::Robot *robot = placementFrameBlue->add_robots();
                        robot->set_robot_id(x);
                        if (x==0)
                        {
                            robot->set_x(-0.7);
                            robot->set_y(0.0);
                            robot->set_orientation(0.0);
                        }
                        if (x==1)
                        {
                            robot->set_x(0.31);
                            robot->set_y(0.01);
                            robot->set_orientation(0.0);
                        }
                        if (x==2)
                        {
                            robot->set_x(-0.25);
                            robot->set_y(-0.4);
                            robot->set_orientation(0.0);
                        }

                    }

                    placementCommandBlue.set_allocated_world(placementFrameBlue);

                    // Sending blue
                    std::string msgBlue;
                    placementCommandBlue.SerializeToString(&msgBlue);
                    if(replacerSocket->write(msgBlue.c_str(), msgBlue.length()) == -1){
                        std::cout << "[Example] Failed to write to replacer socket: " << replacerSocket->errorString().toStdString() << std::endl;
                    }
                }

            }



            if (packet.has_frame())
            {
                fira_message::Frame detection = packet.frame();

                // Quantidade de robos em campo
                int robots_blue_n = detection.robots_blue_size();
                int robots_yellow_n = detection.robots_yellow_size();
                
                double width, length;
                width = 1.3 / 2.0;
                length = 1.7 / 2.0;

                // Informacoes da bola
                fira_message::Ball ball = detection.ball();
                ball.set_x((length + ball.x()) * 100);
                ball.set_y((width + ball.y()) * 100);
//                printf("-Ball:  POS=<%9.2f,%9.2f> \n", ball.x(), ball.y());

                // TIME AZUL
                for (int i = 0; i < robots_blue_n; i++)
                {
                    fira_message::Robot robot = detection.robots_blue(i); // Detecção do robo
                    robot.set_x((length + robot.x()) * 100); // Convertendo para centimetros
                    robot.set_y((width + robot.y()) * 100);
                    robot.set_orientation(to180range(robot.orientation()));

                    if(game_on)
                    {
                        if(!Team_UFRBots)
                        {
                            // Se for o robo 0
                            if(i == 0)
                            {
                                if(ball.x() <= 45 && ball.y() >= 44 && ball.y() <= 86)
                                {
                                    Objective defensor = defineObjective(robot, ball);
                                    PID(robot, defensor, i, commandClient, Team_UFRBots, 5);
                                }

                                else if(ball.y() > 86 && ball.x() > 45)
                                {
                                    Objective parado = Objective(20, 86, 0);
                                    PID(robot, parado, i, commandClient, Team_UFRBots, 0);
                                }

                                else if(ball.y() < 44 && ball.x() > 45)
                                {
                                    Objective parado = Objective(20, 44, 0);
                                    PID(robot, parado, i, commandClient, Team_UFRBots, 0);
                                }

                                else if(ball.x() > 45 && ball.y() >= 44 && ball.y() <= 86)
                                {
                                    Objective parado = Objective(20, ball.y(), 0);
                                    PID(robot, parado, i, commandClient, Team_UFRBots, 0);
                                }
                                else
                                {
                                    if(ball.y() > 65)
                                    {
                                        Objective parado = Objective(20, 86, 0);
                                        PID(robot, parado, i, commandClient, Team_UFRBots, 0);
                                    }
                                    if(ball.y() <= 65)
                                    {
                                        Objective parado = Objective(20, 44, 0);
                                        PID(robot, parado, i, commandClient, Team_UFRBots, 0);
                                    }
                                }
                            }

                            else
                            {

//                                Se robo 1 no campo de ataque e bola no lado esquerdo
                                if(ball.y() >= 64 && ball.x() > 45 && i == 1 && !penalty) {
                                    Objective o = defineObjective(robot, ball);
                                    PID(robot, o, i, commandClient, Team_UFRBots, 10);
                                }
                                else if (penalty && i==1){
                                    printf("TAMO AQUI");
                                    commandClient.sendCommand(-30, 30, Team_UFRBots, i);
                                    penalty = false;
                                }

//                                Se robo 2 no campo de ataque e bola no lado direito
                                else if(ball.y() < 64 && ball.x() > 86 && i == 2) {
                                    Objective o = defineObjective(robot, ball);
                                    PID(robot, o, i, commandClient, Team_UFRBots, 10);
                                }

//                                Se robo 1 no campo de defesa e bola no lado direito
                                else if(ball.y() < 64 && ball.x() <= 85 && ball.x() > 30 && i == 1 && !penalty) {
                                    Objective marcadorAvancado = Objective(ball.x(), 96, 0);
                                    PID(robot, marcadorAvancado, i, commandClient, Team_UFRBots, 0);
                                }

//                                Se robo 1 no campo de ataque e bola no lado direito
                                else if(ball.y() < 64 && ball.x() > 85 && i == 1 && !penalty) {
                                    Objective marcadorAvancado = Objective((ball.x()-20), 96, 0);
                                    PID(robot, marcadorAvancado, i, commandClient, Team_UFRBots, 0);
                                }

//                                Se robo 2 no campo de defesa e bola no lado esquerdo
                                else if(ball.y() >= 64 && ball.x() <= 85 && i == 2) {
                                    Objective marcadorRecuado = Objective(ball.x(), 32, 0);
                                    PID(robot, marcadorRecuado, i, commandClient, Team_UFRBots, 0);
                                }

//                                Se robo 2 no campo de ataque e bola no lado esquerdo
//                                mas antes da pequena area
                                else if(ball.y() >= 64 && ball.x() > 85 && ball.x() <= 125 && i == 2) {
                                    Objective marcadorRecuado = Objective((ball.x()-20), 32, 0);
                                    PID(robot, marcadorRecuado, i, commandClient, Team_UFRBots, 0);
                                }

//                                Se robo 2 no campo de ataque e bola no lado esquerdo
//                                mas na pequena area do adversario
//                                else if(ball.y() >= 64 && ball.x() > 125 && i == 2) {
//                                    Objective o = defineObjective(robot, ball);
//                                    PID(robot, o, i, commandClient, Team_UFRBots);
//                                }

//                                Condicao de movimento para freeball quadrante Q2
                                else if(ball.x() >= 45 && ball.x() <= 84 && ball.y() > 65 && i == 1 && !penalty) {
                                    Objective o = defineObjective(robot, ball);
                                    PID(robot, o, i, commandClient, Team_UFRBots, 0);
                                }

//                                Condicao de movimento para freeball quadrante Q3
                                else if(ball.x() >= 45 && ball.x() <= 84 && ball.y() <= 65 && i == 2 ) {
                                    Objective o = defineObjective(robot, ball);
                                    PID(robot, o, i, commandClient, Team_UFRBots, 0);
                                }

                                else
                                {
                                    commandClient.sendCommand(0, 0, Team_UFRBots, i);
                                }
                            }
                        }
                     }

                    else
                    {
                        commandClient.sendCommand(0, 0, Team_UFRBots, i);
                    }

                }


                // TIME AMARELO
                for (int i = 0; i < robots_yellow_n; i++)
                {
                    fira_message::Robot robot = detection.robots_yellow(i); // Detecção do robo
                    robot.set_x((length + robot.x()) * 100); // Convertendo para centimetros
                    robot.set_y((width + robot.y()) * 100);
                    robot.set_orientation(to180range(robot.orientation()));
//                    printf("-Ball:  POS=<%9.2f,%9.2f> \n", ball.x(), ball.y());

                if(game_on)
                {
                    if(Team_UFRBots)
                    {

                        if(i == 0)
                        {
                            if(ball.x() > 135 && ball.y() >= 47 && ball.y() <= 93)
                            {
                                Objective defensor = defineObjectiveYellow(robot, ball);
                                PID(robot, defensor, i, commandClient, Team_UFRBots, 0);
                            }

                            if(ball.y() > 88 && ball.x() < 135)
                            {
                                Objective parado = Objective(153, 85, 0);
                                PID(robot, parado, i, commandClient, Team_UFRBots, 0);
                            }

                            if(ball.y() < 52 && ball.x() < 135)
                            {
                                Objective parado = Objective(153, 55, 0);
                                PID(robot, parado, i, commandClient, Team_UFRBots, 0);
                            }

                            if(ball.y() >= 52 && ball.y() <= 88 && ball.x() < 135)
                            {
                                Objective parado = Objective(153, ball.y(), 0);
                                PID(robot, parado, i, commandClient, Team_UFRBots, 0);
                            }
                        }


                        else
                        {

                            if(ball.y() >= 64 && i == 1) {
                                Objective o = defineObjectiveYellow(robot, ball);
                                PID(robot, o, i, commandClient, Team_UFRBots, 0);
                            }

                            if(ball.y() < 64 && ball.x() < 86 && i == 2) {
                                Objective o = defineObjectiveYellow(robot, ball);
                                PID(robot, o, i, commandClient, Team_UFRBots, 0);
                            }

                            if(ball.y() < 65 && i == 1) {
                                Objective marcadorAvancado = Objective(85, ball.y(), 0);
                                PID(robot, marcadorAvancado, i, commandClient, Team_UFRBots, 0);
                            }

                            if(ball.y() >= 65 && ball.x() < 86 && i == 2) {
                                Objective marcadorRecuado = Objective(100, ball.y(), 0);
                                PID(robot, marcadorRecuado, i, commandClient, Team_UFRBots, 0);
                            }

                            if(ball.x() >= 86 && i == 2) {
                                Objective o = defineObjectiveYellow(robot, ball);
                                PID(robot, o, i, commandClient, Team_UFRBots, 0);
                            }
                        }

                        // Se for o robo 0
//                        if(i == 0)
//                        {
//                            if(ball.x() > 120)
//                            {
//                                Objective defensor = defineObjective(robot, ball, Team_UFRBots);
//                                PID(robot, defensor, 0, commandClient, Team_UFRBots);
//                            }
//                            else
//                            {
//                                Objective parado = Objective(155, 65, 0);
//                                PID(robot, parado, 0, commandClient, Team_UFRBots);
//                            }
//                        }
//                        // Caso contrário, robo 1 e 2
//                        else
//                        {
//                            Objective o = defineObjective(robot, ball, Team_UFRBots);

//                            if(ball.y() >= 65 && i == 1) {
//                                PID(robot, o, i, commandClient, Team_UFRBots);
//                            }
//                            if(ball.y() < 65 && i == 2) {
//                                PID(robot, o, i, commandClient, Team_UFRBots);
//                            }
//                            if(ball.x() >= 86 && i == 2) {
//                                PID(robot, o, i, commandClient, Team_UFRBots);
//                            }
//                        }
//                    }
//                }
//                    else
//                    {
//                        commandClient.sendCommand(0, 0, Team_UFRBots, i);
//                    }



                    }
                }

            }
        }
        }
    }


    return 0;
}
