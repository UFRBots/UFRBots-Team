//author  Renato Sousa, 2018
#include <QtNetwork>
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"

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

void printRobotInfo(const fira_message::Robot &robot)
{
    printf("ID=%3d \n", robot.robot_id());

    printf(" POS=<%9.2f,%9.2f> \n", robot.x(), robot.y());
    printf(" VEL=<%9.2f,%9.2f> \n", robot.vx(), robot.vy());

    printf("ANGLE=%6.3f \n", robot.orientation());
    printf("ANGLE VEL=%6.3f \n", robot.vorientation());
}

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

void PID(fira_message::Robot robot, Objective objective, int index, GrSim_Client &commandClient, bool Team_UFRBots)
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

    double baseSpeed = 30;

    // Normalize
    motorSpeed = motorSpeed > 30 ? 30 : motorSpeed;
    motorSpeed = motorSpeed < -30 ? -30 : motorSpeed;

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
    //calculo da distancia entre o jogador e a bola, por meio do módulo do vetor diferença!
    if (robot.x() < ball.x())
    { //se o robô está atrás da bola
        estado = APROXIMA;

        if ((robot.y() < ball.y() - 5 || robot.y() > ball.y() + 5) && robot.x() >= 22)
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

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    // Robos da equipe amarelo = true;
    // Robos da equipe azul = false;
    bool Team_UFRBots = false;
    bool Team_Cruzeiro = true;

    // IP do simulador
    RoboCupSSLClient visionClient("224.0.0.1", 10002);
    visionClient.open(false);

    GrSim_Client commandClient("127.0.0.1", 20011);

    // Pacotes de mensagens
    fira_message::sim_to_ref::Environment packet;

    while (true)
    {
        if (visionClient.receive(packet))
        {
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
                printf("-Ball:  POS=<%9.2f,%9.2f> \n", ball.x(), ball.y());
                
                // TIME AZUL
                for (int i = 0; i < robots_blue_n; i++)
                {
                    fira_message::Robot robot = detection.robots_blue(i); // Detecção do robo
                    robot.set_x((length + robot.x()) * 100); // Convertendo para centimetros
                    robot.set_y((width + robot.y()) * 100);
                    robot.set_orientation(to180range(robot.orientation()));
//                    printf("-Robot(B) (%2d/%2d): ", i + 1, robots_blue_n);
                    if(!Team_UFRBots)
                    {
                        // Se for o robo 0
                        if(i == 0)
                        {
                            if(ball.x() < 50)
                            {
                                Objective defensor = defineObjective(robot, ball);
                                PID(robot, defensor, 0, commandClient, Team_UFRBots);
                            }
                            else
                            {
                                Objective parado = Objective(15, 65, 0);
                                PID(robot, parado, 0, commandClient, Team_UFRBots);
                            }
                        }
                        // Caso contrário, robo 1 e 2
                        else
                        {
                            Objective o = defineObjective(robot, ball);

                            if(ball.y() >= 65 && i == 1) {
                                PID(robot, o, i, commandClient, Team_UFRBots);
                            }
                            if(ball.y() < 65 && i == 2) {
                                PID(robot, o, i, commandClient, Team_UFRBots);
                            }
                            if(ball.x() <= 76 && i == 2) {
                                PID(robot, o, i, commandClient, Team_UFRBots);
                            }
                        }
                    }
                  }



                    // TIME AMARELO
                    for (int i = 0; i < robots_yellow_n; i++)
                    {
                        fira_message::Robot robot = detection.robots_yellow(i); // Detecção do robo
                        robot.set_x((length + robot.x()) * 100); // Convertendo para centimetros
                        robot.set_y((width + robot.y()) * 100);
                        robot.set_orientation(to180range(robot.orientation()));
    //                    printf("-Robot(B) (%2d/%2d): ", i + 1, robots_yellow_n);
                        if(Team_Cruzeiro)
                        {
                            // Se for o robo 0
                            if(i == 0)
                            {
                                if(ball.x() > 120)
                                {
                                    Objective defensor = defineObjective(robot, ball);
                                    PID(robot, defensor, 0, commandClient, Team_Cruzeiro);
                                }
                                else
                                {
                                    Objective parado = Objective(155, 65, 0);
                                    PID(robot, parado, 0, commandClient, Team_Cruzeiro);
                                }
                            }
                            // Caso contrário, robo 1 e 2
                            else
                            {
                                Objective o = defineObjective(robot, ball);

                                if(ball.y() >= 65 && i == 1) {
                                    PID(robot, o, i, commandClient, Team_Cruzeiro);
                                }
                                if(ball.y() < 65 && i == 2) {
                                    PID(robot, o, i, commandClient, Team_Cruzeiro);
                                }
                                if(ball.x() >= 74 && i == 2) {
                                    PID(robot, o, i, commandClient, Team_Cruzeiro);
                                }
                            }
                        }



                    //Blue robot info:
                    for (int i = 0; i < robots_blue_n; i++)
                    {
                        fira_message::Robot robot = detection.robots_blue(i);
                        printf("-Robot(B) (%2d/%2d): ", i + 1, robots_blue_n);
                        printRobotInfo(robot);
                    }
                }
            }


            //see if packet contains geometry data:
            if (packet.has_field())
            {
                printf("-[Geometry Data]-------\n");

                const fira_message::Field &field = packet.field();
                printf("Field Dimensions:\n");
                printf("  -field_length=%f (mm)\n", field.length());
                printf("  -field_width=%f (mm)\n", field.width());
                printf("  -goal_width=%f (mm)\n", field.goal_width());
                printf("  -goal_depth=%f (mm)\n", field.goal_depth());
            }
        }
    }

    return 0;
}