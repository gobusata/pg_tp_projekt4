// projekt4.cpp : Defines the entry point for the application.
//

#include <windows.h>
#include <windowsx.h>
#include <gdiplus.h>
#include <vector>
#include <chrono>
#include <ShObjIdl.h>
#include <oleacc.h>
#include <strsafe.h>
#include <sstream>
using namespace Gdiplus;

#include "framework.h"
#include "projekt4.h"
#include "Triangle.h"
#include "Robot.h"
#include "ProjektLogs.h"
#include "ProjektConstraint.h"
#include "ProjektConstraintPlane.h"
#include "ProjektConstraintNoPenetration.h"
#include "UniversalConvexShape.h"

#pragma comment(lib, "gdiplus.lib")

#define MAX_LOADSTRING 100

// Global Variables:
class AppState;
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name

// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int, AppState*);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);
void myOnPaint(HDC&, AppState*);

// button size
const SIZE BTN_SIZE{ 110, 40 }, PADDING = { 2, 2 };

HWND m_hwnd;
BOOL end = FALSE;
//appplication state
class AppState
{
private:   
    //gravity
    PointF gravity{ 0, 5e-4 };

    std::vector<ProjektConvexBody> shapes;
    std::vector<std::shared_ptr<ProjektConstraint>> constraints;
    std::vector<std::shared_ptr<ProjektConstraintNoPenetration>> noPenetrationConstraints;
    std::vector<std::shared_ptr<ProjektConstraintPlane>> planeConstraints;
    bool is_robot = false, run_script = false;
    Robot* robot = nullptr;
    int stage;
    Vector2f shapes_intersection;
    std::vector<PointF> collision_points{};

public:
    //pens and brushes in basic colors
    std::vector<Pen*> pens;
    SolidBrush redBrush, greenBrush, blueBrush, whiteBrush, blackBrush;
    Pen redPen, greenPen, bluePen, whitePen, blackPen;
    bool pause_simulation = false;
    //rects
    Rect client_rect;
    //animation rect
    RectF ar;
    HWND list_box;
    
    void print(RobotPosition rp, TCHAR* text)
    {
        TCHAR text1[27], text2[12];
        StringCbPrintf(text1, ARRAYSIZE(text1), TEXT("(%d, %d) "), rp.pos.X, rp.pos.Y);
        switch (rp.robotCommand)
        {
        case rc_stop:
            StringCbPrintf(text2, ARRAYSIZE(text2), TEXT("stop"));
            break;
        case rc_do_nothing:
            StringCbPrintf(text2, ARRAYSIZE(text2), TEXT(""));
            break;
        case rc_catch:
            StringCbPrintf(text2, ARRAYSIZE(text2), TEXT("catch"));
            break;
        case rc_release:
            StringCbPrintf(text2, ARRAYSIZE(text2), TEXT("drop"));
            break;
        }
        StringCchCat(text1, 27, text2);
        StringCchCopy(text, 27, text1);
    }

    std::vector<RobotPosition> list_box_trajectory;

    AppState():
        //initialize pens and brushes
        redBrush(Color(215, 20, 20)), greenBrush(Color(20, 215, 20)), blueBrush(Color(20, 20, 215)), whiteBrush(Color(200, 200, 200)), blackBrush(Color(0, 0, 0)),
        redPen(&redBrush, 2), greenPen(&greenBrush, 2), bluePen(&blueBrush, 2), whitePen(&whiteBrush, 2), blackPen(&blackBrush, 1),
        pens{ &redPen, &greenPen, &bluePen, &blackPen },
        //initialize rects
        client_rect(0, 0, 750, 530), ar(10, 10, 480, 370),
        list_box_trajectory(), shapes_intersection{}
    {
        dbginit();
        for (auto & p : pens)
        {
            p->SetLineCap(LineCapRound, LineCapRound, DashCapRound);
        }
    }

    void draw(Graphics* graphics)
    {
        
        graphics->DrawRectangle(&bluePen, ar);
        graphics->FillRectangle(&whiteBrush, ar);

        for (PointF p: collision_points)
        {
            graphics->FillRectangle(&this->blackBrush, (REAL)p.X - 2.5, (REAL)p.Y - 2.5, (REAL)5., (REAL)5.);
        }

        for (ProjektConvexBody& s : shapes)
        {
            s.draw(graphics, &bluePen, nullptr);
        }

        for (std::shared_ptr<const ProjektConstraintNoPenetration>  con : noPenetrationConstraints)
        {
            switch (con->intersection.simpstate)
            {
            case GjkSimplex::line:
                graphics->DrawLine(&blackPen, ev2gp(con->intersection.simplex_vertices[0]), ev2gp(con->intersection.simplex_vertices[1]));
                graphics->DrawLine(&blackPen, ev2gp(con->intersection.simplex_vertices[3]), ev2gp(con->intersection.simplex_vertices[4]));
                break;
            case GjkSimplex::triangle:
                graphics->DrawLine(&blackPen, ev2gp(con->intersection.simplex_vertices[0]), ev2gp(con->intersection.simplex_vertices[1]));
                graphics->DrawLine(&blackPen, ev2gp(con->intersection.simplex_vertices[1]), ev2gp(con->intersection.simplex_vertices[2]));
                graphics->DrawLine(&blackPen, ev2gp(con->intersection.simplex_vertices[2]), ev2gp(con->intersection.simplex_vertices[0]));
                graphics->DrawLine(&blackPen, ev2gp(con->intersection.simplex_vertices[3]), ev2gp(con->intersection.simplex_vertices[4]));
                graphics->DrawLine(&blackPen, ev2gp(con->intersection.simplex_vertices[4]), ev2gp(con->intersection.simplex_vertices[5]));
                graphics->DrawLine(&blackPen, ev2gp(con->intersection.simplex_vertices[5]), ev2gp(con->intersection.simplex_vertices[3]));
                break;
            }
            for (const ProjektConstraintNoPenetration::SubConstraint& sc : con->subconstraints)
            {
                if (sc.active)
                {
                    Vector2f tmp{ con->pointOfContact(sc) };
                    graphics->FillRectangle(&this->redBrush, (REAL)tmp[0] - 1.5, (REAL)tmp[1] - 1.5, (REAL)3., (REAL)3.);
                }
            }
        }

        for (std::shared_ptr<const ProjektConstraintPlane> con : planeConstraints)
        {
            for (const ProjektConstraintPlane::SubConstraint& sc : con->subconstraints)
            {
                if (sc.active)
                {
                    Vector2f tmp{ con->pointOfContact(sc) };
                    graphics->FillRectangle(&this->redBrush, (REAL)tmp[0] - 1.5, (REAL)tmp[1] - 1.5, (REAL)3., (REAL)3.);
                }
            }
        }
        if (is_robot) robot->draw(graphics);
    }

    void update(REAL dt)
    {
        for (std::vector<ProjektConvexBody>::iterator s = shapes.begin(); s != shapes.end(); s++)
            s->updateVel(dt);
        float error = 0;
        int i = 0;
        for (std::vector<std::shared_ptr<ProjektConstraint>>::iterator c = constraints.begin(); c != constraints.end(); c++)
            (*c)->activateImpulse();
        for (std::shared_ptr<ProjektConstraint> & c : constraints)
           c->applyAccImpulse();
        for (i= 0; i<20; i++)
        {
            error = 0;
            for (std::vector<std::shared_ptr<ProjektConstraint>>::iterator c = constraints.begin(); c != constraints.end(); c++)
                error = max((*c)->calcApplyImpulse(dt), error);
            if (i >= 79)
            {
                dbgmsg("NoPenetrationConstraints:");
                for (auto coni : noPenetrationConstraints)
                {
                    for (auto sci : coni->subconstraints)
                    {
                        dbgmsg("j_mat = {}", sci.jmat);
                    }
                }
                dbgmsg("PlaneConstraints:");
                for (auto coni : planeConstraints)
                {
                    for (auto sci : coni->subconstraints)
                    {
                        dbgmsg("j_mat = {}", sci.jn);
                    }
                }
            }
            if (error < 0.05)
                break;
        }
        if (i > 0)
            dbgmsg("Number of loops: {}\t\t error = {:.4e}", i, error);
        for (std::vector<std::shared_ptr<ProjektConstraint>>::iterator c = constraints.begin(); c != constraints.end(); c++)
            (*c)->storeAccImpulse();
        for (std::vector<ProjektConvexBody>::iterator s = shapes.begin(); s != shapes.end(); s++)
            s->updatePos(dt);
        
        if (is_robot)
        {
            robot->update(dt);
            if (run_script) script(dt);
        }
       
    }

    void demo1()
    {
        shapes.clear();
        constraints.clear();
        planeConstraints.clear();
        noPenetrationConstraints.clear();
        gravity.Y = ProjektConvexBody::gravity;
        //shapes.push_back(ProjektConvexBody(Vector3f{ 260, ar.GetBottom() - 21, 0 }, Vector3f{ 0, 0, 0 },
        //    std::vector<Vector2f>({ Vector2f{ 20.00, 0.00 }, Vector2f{ 18.48, 7.65 }, Vector2f{ 14.14, 14.14 }, Vector2f{ 7.65, 18.48 }, 
        //        Vector2f{ -0.00, 20.00 }, Vector2f{ -7.65, 18.48 }, Vector2f{ -14.14, 14.14 }, Vector2f{ -18.48, 7.65 }, 
        //        Vector2f{ -20.00, -0.00 }, Vector2f{ -18.48, -7.65 }, Vector2f{ -14.14, -14.14 }, Vector2f{ -7.65, -18.48 }, 
        //        Vector2f{ 0.00, -20.00 }, Vector2f{ 7.65, -18.48 }, Vector2f{ 14.14, -14.14 } }),
        //    1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 300, ar.GetBottom() - 41, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{20, 40}, Vector2f{-20, 40}, Vector2f{-20, -40}, Vector2f{20, -40} }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 300, ar.GetBottom() - 100, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{40, 5}, Vector2f{40, -5}, Vector2f{-40, -5}, Vector2f{-40, 5} }), 1.f, 1.f));
        for (std::vector<ProjektConvexBody>::iterator pcb1 = shapes.begin(); pcb1 != shapes.end(); pcb1++)
        {
            constraints.push_back(std::make_shared<ProjektConstraintPlane>(*pcb1, Vector2f{ 0.f , 1.f }, Vector2f{ 0.5*(ar.GetLeft() + ar.GetRight()), ar.GetBottom() }));
            planeConstraints.push_back(std::reinterpret_pointer_cast<ProjektConstraintPlane>(constraints.back()));
            for (auto pcb2 = pcb1+1; pcb2 != shapes.end(); pcb2++)
                constraints.push_back(std::make_shared<ProjektConstraintNoPenetration>(*pcb1, *pcb2));
        }

    }

    void demo2()
    {
        shapes.clear();
        constraints.clear();
        planeConstraints.clear();
        noPenetrationConstraints.clear();
        gravity.Y = ProjektConvexBody::gravity;
        shapes.push_back(ProjektConvexBody(Vector3f{ 100, ar.GetBottom() - 42, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 40, 40 }, Vector2f{ -40, 40 }, Vector2f{ -40, -40 }, Vector2f{ 40, -40 } }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 100, ar.GetBottom() - 124, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 40, 40 }, Vector2f{ -40, 40 }, Vector2f{ -40, -40 }, Vector2f{ 40, -40 } }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 100, ar.GetBottom() - 206, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 40, 40 }, Vector2f{ -40, 40 }, Vector2f{ -40, -40 }, Vector2f{ 40, -40 } }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 170, ar.GetBottom() - 22, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 20, 20 }, Vector2f{ -20, 20 }, Vector2f{ -20, -20 }, Vector2f{ 20, -20 } }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 170, ar.GetBottom() - 64, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 20, 20 }, Vector2f{ -20, 20 }, Vector2f{ -20, -20 }, Vector2f{ 20, -20 } }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 170, ar.GetBottom() - 106, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 20, 20 }, Vector2f{ -20, 20 }, Vector2f{ -20, -20 }, Vector2f{ 20, -20 } }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 220, ar.GetBottom() - 12, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 10, 10 }, Vector2f{ -10, 10 }, Vector2f{ -10, -10 }, Vector2f{ 10, -10 } }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 220, ar.GetBottom() - 34, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 10, 10 }, Vector2f{ -10, 10 }, Vector2f{ -10, -10 }, Vector2f{ 10, -10 } }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 220, ar.GetBottom() - 56, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 10, 10 }, Vector2f{ -10, 10 }, Vector2f{ -10, -10 }, Vector2f{ 10, -10 } }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 300, ar.GetBottom() - 56, 0 }, Vector3f{ -0.3, -0.3, 0 },
            std::vector<Vector2f>({ Vector2f{ 10, 10 }, Vector2f{ -10, 10 }, Vector2f{ -10, -10 }, Vector2f{ 10, -10 } }), 1.f, 1.f));
        for (std::vector<ProjektConvexBody>::iterator pcb1 = shapes.begin(); pcb1 != shapes.end(); pcb1++)
        {
            constraints.push_back(std::make_shared<ProjektConstraintPlane>(*pcb1, Vector2f{ 0.f , 1.f }, Vector2f{ 0.5 * (ar.GetLeft() + ar.GetRight()), ar.GetBottom() }));
            planeConstraints.push_back(std::reinterpret_pointer_cast<ProjektConstraintPlane>(constraints.back()));
            for (auto pcb2 = pcb1 + 1; pcb2 != shapes.end(); pcb2++)
                constraints.push_back(std::make_shared<ProjektConstraintNoPenetration>(*pcb1, *pcb2));
        }
    }

    void demo3()
    {
        shapes.clear();
        constraints.clear();
        planeConstraints.clear();
        noPenetrationConstraints.clear();
        gravity.Y = ProjektConvexBody::gravity;
        shapes.push_back(ProjektConvexBody(Vector3f{ 100, ar.GetBottom() - 42, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 10, 10 }, Vector2f{ -10, 10 }, Vector2f{ -10, -10 }, Vector2f{ 10, -10 } }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 100, ar.GetBottom() - 126, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 10, 10 }, Vector2f{ -10, 10 }, Vector2f{ -10, -10 }, Vector2f{ 10, -10 } }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 100, ar.GetBottom() - 210, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 10, 10 }, Vector2f{ -10, 10 }, Vector2f{ -10, -10 }, Vector2f{ 10, -10 } }), 1.f, 1.f));
        for (std::vector<ProjektConvexBody>::iterator pcb1 = shapes.begin(); pcb1 != shapes.end(); pcb1++)
        {
            constraints.push_back(std::make_shared<ProjektConstraintPlane>(*pcb1, Vector2f{ 0.f , 1.f }, Vector2f{ 0.5 * (ar.GetLeft() + ar.GetRight()), ar.GetBottom() }));
            planeConstraints.push_back(std::reinterpret_pointer_cast<ProjektConstraintPlane>(constraints.back()));
            for (auto pcb2 = pcb1 + 1; pcb2 != shapes.end(); pcb2++)
                constraints.push_back(std::make_shared<ProjektConstraintNoPenetration>(*pcb1, *pcb2));
            
        }
    }

    void demo4()
    {
        shapes.clear();
        constraints.clear();
        planeConstraints.clear();
        noPenetrationConstraints.clear();
        gravity.Y = ProjektConvexBody::gravity;
        shapes.push_back(ProjektConvexBody(Vector3f{ 100, ar.GetBottom() - 40, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 40, 40 }, Vector2f{ -40, 40 }, Vector2f{ -40, -40 }, Vector2f{ 40, -40 } }), 1.f, 1.f));
        shapes.push_back(ProjektConvexBody(Vector3f{ 100, ar.GetBottom() - 134, 0 }, Vector3f{ 0, 0, 0 },
            std::vector<Vector2f>({ Vector2f{ 40, 40 }, Vector2f{ -40, 40 }, Vector2f{ -40, -40 }, Vector2f{ 40, -40 } }), 1.f, 1.f));
        for (std::vector<ProjektConvexBody>::iterator pcb1 = shapes.begin(); pcb1 != shapes.end(); pcb1++)
        {
            constraints.push_back(std::make_shared<ProjektConstraintPlane>(*pcb1, Vector2f{ 0.f , 1.f }, Vector2f{ 0.5 * (ar.GetLeft() + ar.GetRight()), ar.GetBottom() }));
            planeConstraints.push_back(std::reinterpret_pointer_cast<ProjektConstraintPlane>(constraints.back()));
            for (auto pcb2 = pcb1 + 1; pcb2 != shapes.end(); pcb2++)
            {
                std::shared_ptr<ProjektConstraintNoPenetration> pcnp = std::make_shared<ProjektConstraintNoPenetration>(*pcb1, *pcb2);
                constraints.push_back(pcnp);
                noPenetrationConstraints.push_back(pcnp);
            }
                
        }
    }

    inline void SetRobotArm1(REAL a)
    {
        if (is_robot)
        {
            robot->set_tAngle1(a - 90);
        }
    }

    inline void SetRobotArm2(REAL a)
    {
        if (is_robot)
        {
            robot->set_tAngle2(a - 180);
        }
    }

    inline void SetRobotPosition(Point p)
    {
        if (this->is_robot)
        {
            this->robot->set_tPosition(PointF(p.X, p.Y));
        }
    }

    inline void SetRobotTrajectory(const std::vector<RobotPosition>& trajectory)
    {
        this->robot->enter_trajectory(trajectory);
    }

    void on_catch()
    {
        if (is_robot)
        {
            if (robot->catched_triangle == nullptr)
            {

            }
            else
            {
                robot->catched_triangle = nullptr;
            }
           
        }
    }
    
    void start_script()
    {
        run_script = true;

    }

    void stop_script()
    {
        run_script = false;
    }

    void script(REAL dt)
    {
       

        std::vector<PointF> t{ {60, 60}, {50, 360}, {200, 200}, {400, 360}, {400, 360},
             {200, 200}, {250, 360}, {250, 200}, {350, 360}, {350, 360},
             {200, 200}, {150, 340}, {200, 200}, {375, 280}, {375, 300}, {375, 305}, {375, 310},
            {200, 200}

        };
        std::vector<int> t2{ 1, 6, 11, 4, 9, 16 };
        static REAL t1_ms, t2_ms;
        PointF dest, start;
        if (is_robot && run_script)
        {
            if (stage == 0)
            {
                stage = 1;
                t1_ms = t2_ms = 2000;
            }

            if (stage < t.size())
            {
                if (t1_ms > 0)
                {
                    t1_ms -= dt;
                    robot->set_postion(t[stage] + (t[stage - 1] - t[stage]) * (t1_ms / t2_ms));
                }
                else
                {
                    for (auto i : t2)
                    {
                        if (stage == i) on_catch();
                    }
                    stage++;
                    t1_ms = t2_ms;
                }
            }
           
  
        }
    }

    inline void follow_line(PointF start, PointF dest, float & t1_ms, float t2_ms, int & stage, float dt)
    {
       
    }

    ~AppState()
    {
        if (is_robot) delete robot;
    }
};

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    // Initialize global strings
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_PROJEKT4, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    ULONG_PTR token;
    GdiplusStartupInput gdiinput;
    GdiplusStartupOutput gdioutput;
    GdiplusStartup(&token, &gdiinput, &gdioutput);

    AppState *appState = new AppState;

    // Perform application initialization:
    if (!InitInstance (hInstance, nCmdShow, appState))
    {
        return FALSE;
    }

    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_PROJEKT4));

    MSG msg;

    // Main message loop:
    InvalidateRect(m_hwnd, NULL, TRUE);

    std::chrono::steady_clock sc;
    std::chrono::milliseconds interval(50);
    int count_frames1 = 0, count_frames2 = 0, fps;
    std::chrono::steady_clock::time_point fps_start = sc.now();
    std::chrono::steady_clock::time_point one_loop_start = sc.now();
    std::chrono::steady_clock::time_point start_count = sc.now();
    std::chrono::milliseconds update_interval(5);
    do
    {
        std::chrono::microseconds dt(std::chrono::duration_cast<std::chrono::microseconds>(sc.now() - one_loop_start));
        if (dt > update_interval)
        {
            std::chrono::steady_clock::time_point temp = sc.now();
            float dt_ = (dt.count() > 20e3 ? 5.0f : float(dt.count()) * 0.25e-3);
            if(!appState->pause_simulation)
              appState->update(dt_);
            //dbgmsg("period: {}us, interval: {}ms", dt.count(), interval.count());
            one_loop_start = temp;
        }

        if (sc.now() - fps_start > interval)
        {
            
            fps_start = sc.now();
            RECT t{ 0, 0, 100, 100 };
            InvalidateRect(m_hwnd, NULL, TRUE);
        }
        
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE) && !end && sc.now() - fps_start < interval)
        {
            if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
            {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
        } 
        
    } while (!end);

    

    delete appState;
    GdiplusShutdown(token);
    return (int) msg.wParam;
}



//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style          = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    wcex.lpfnWndProc    = WndProc;
    wcex.cbClsExtra     = 0;
    wcex.cbWndExtra     = 0;
    wcex.hInstance      = hInstance;
    wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_PROJEKT4));
    wcex.hCursor        = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground  = (HBRUSH)(COLOR_MENU+1);
    wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_PROJEKT4);
    wcex.lpszClassName  = szWindowClass;
    wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    return RegisterClassExW(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow, AppState* appstate)
{
   hInst = hInstance; // Store instance handle in our global variable

   DWORD ws_style = WS_CLIPCHILDREN | WS_OVERLAPPED | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SYSMENU | WS_CAPTION | WS_VISIBLE | WS_BORDER;
   Rect window_rect(appstate->client_rect);
   AdjustWindowRectEx(reinterpret_cast<LPRECT>(&window_rect), ws_style, TRUE, WS_EX_OVERLAPPEDWINDOW);
   //appstate->client_rect.Offset(window_rect.GetLeft(), window_rect.GetTop());

   m_hwnd = CreateWindowEx(WS_EX_OVERLAPPEDWINDOW, szWindowClass, szTitle, ws_style,
      CW_USEDEFAULT, CW_USEDEFAULT, window_rect.GetRight() - window_rect.GetLeft(), window_rect.GetBottom() - window_rect.GetTop(),
       nullptr, nullptr, hInstance, reinterpret_cast<void*>(appstate));

   GetClientRect(m_hwnd, reinterpret_cast<LPRECT>(&appstate->client_rect));
   appstate->ar = RectF{ REAL(appstate->client_rect.GetLeft()+PADDING.cx), REAL(appstate->client_rect.GetTop()+PADDING.cy),
       REAL(appstate->client_rect.GetRight() - appstate->client_rect.GetLeft() - 2*PADDING.cx - 2*BTN_SIZE.cx),
       REAL(appstate->client_rect.GetBottom() - 2 * BTN_SIZE.cy - appstate->client_rect.GetTop() - 2 * PADDING.cy)};

   CreateWindow(WC_BUTTON, TEXT("&demo 1"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetLeft(), appstate->client_rect.GetBottom()-2*BTN_SIZE.cy+PADDING.cy,
       BTN_SIZE.cx-2*PADDING.cx, BTN_SIZE.cy-2*PADDING.cy,
       m_hwnd, (HMENU)ID_DEMO1, hInstance, NULL);
   
   CreateWindow(WC_BUTTON, TEXT("demo 2"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetLeft()+BTN_SIZE.cx, appstate->client_rect.GetBottom() - 2 * BTN_SIZE.cy + PADDING.cy,
       BTN_SIZE.cx - 2 * PADDING.cx, BTN_SIZE.cy - 2 * PADDING.cy,
       m_hwnd, (HMENU)ID_DEMO2, hInstance, NULL);
   
   CreateWindow(WC_BUTTON, TEXT("demo 3"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetLeft()+2*BTN_SIZE.cx, appstate->client_rect.GetBottom() - 2 * BTN_SIZE.cy + PADDING.cy,
       BTN_SIZE.cx - 2 * PADDING.cx, BTN_SIZE.cy - 2 * PADDING.cy,
       m_hwnd, (HMENU)ID_DEMO3, hInstance, NULL);
   
   CreateWindow(WC_BUTTON, TEXT("demo 4"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetLeft()+3*BTN_SIZE.cx, appstate->client_rect.GetBottom() - 2 * BTN_SIZE.cy + PADDING.cy,
       BTN_SIZE.cx - 2 * PADDING.cx, BTN_SIZE.cy - 2 * PADDING.cy,
       m_hwnd, (HMENU)ID_DEMO4, hInstance, NULL);

   HWND hwnd = CreateWindowEx(NULL, TRACKBAR_CLASS, TEXT("robot control 1"), WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
       appstate->client_rect.GetLeft(), appstate->ar.GetBottom() + BTN_SIZE.cy, 2 * BTN_SIZE.cx, BTN_SIZE.cy,
       m_hwnd, (HMENU)ID_ROBOT1, hInstance, NULL);

   SendMessage(hwnd, TBM_SETRANGE, TRUE, MAKELPARAM(0, 180));
   SendMessage(hwnd, TBM_SETPOS, TRUE, 45);
   SendMessage(hwnd, TBM_SETTICFREQ, 10, 0);

   hwnd = CreateWindowEx(NULL, TRACKBAR_CLASS, TEXT("robot control 2"), WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
       appstate->client_rect.GetLeft() + 2*BTN_SIZE.cx, appstate->ar.GetBottom() + BTN_SIZE.cy, 2 * BTN_SIZE.cx, BTN_SIZE.cy,
       m_hwnd, (HMENU)ID_ROBOT2, hInstance, NULL);

   SendMessage(hwnd, TBM_SETRANGE, TRUE, MAKELPARAM(0, 360));
   SendMessage(hwnd, TBM_SETPOS, TRUE, 315);
   SendMessage(hwnd, TBM_SETTICFREQ, 10, 0);

   HWND list_box_hwnd = CreateWindowEx(WS_EX_CLIENTEDGE, WC_LISTBOX, TEXT("list view"), WS_CHILD | LBS_HASSTRINGS | WS_VISIBLE | WS_BORDER,
       appstate->client_rect.GetRight()-2*BTN_SIZE.cx, appstate->client_rect.GetTop()+PADDING.cx,
       2*BTN_SIZE.cx-2*PADDING.cx, appstate->client_rect.GetBottom()-2*BTN_SIZE.cy-appstate->client_rect.GetTop()-2*PADDING.cy,
       m_hwnd, NULL, hInstance, NULL);
   appstate->list_box = list_box_hwnd;

   CreateWindow(WC_BUTTON, TEXT("&usuñ"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetRight()-2*BTN_SIZE.cx, appstate->client_rect.GetBottom()-2*BTN_SIZE.cy+PADDING.cy,
       BTN_SIZE.cx/2-2*PADDING.cx, BTN_SIZE.cy-2*PADDING.cy,
       m_hwnd, (HMENU)ID_TRAJECTORY_DEL, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("&catch"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetRight() - 1.5 * BTN_SIZE.cx, appstate->client_rect.GetBottom() - 2 * BTN_SIZE.cy+PADDING.cy,
       BTN_SIZE.cx / 2 - 2 * PADDING.cx, BTN_SIZE.cy - 2 * PADDING.cy,
       m_hwnd, (HMENU)ID_TRAJECTORY_CATCH, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("&drop"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetRight() - 1 * BTN_SIZE.cx, appstate->client_rect.GetBottom() - 2 * BTN_SIZE.cy+PADDING.cy,
       BTN_SIZE.cx / 2 - 2 * PADDING.cx, BTN_SIZE.cy - 2 * PADDING.cy,
       m_hwnd, (HMENU)ID_TRAJECTORY_REL, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("&stop"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetRight() - 0.5 * BTN_SIZE.cx, appstate->client_rect.GetBottom() - 2 * BTN_SIZE.cy+PADDING.cy,
       BTN_SIZE.cx / 2 - 2 * PADDING.cx, BTN_SIZE.cy - 2 * PADDING.cy,
       m_hwnd, (HMENU)ID_TRAJECTORY_STOP, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("follow"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetRight() - 2 * BTN_SIZE.cx, appstate->client_rect.GetBottom() - 1 * BTN_SIZE.cy+PADDING.cy,
       BTN_SIZE.cx - 2 * PADDING.cx, BTN_SIZE.cy - 2 * PADDING.cy,
       m_hwnd, (HMENU)ID_TRAJECTORY_LOAD, hInstance, NULL);

   if (!m_hwnd)
   {
      return FALSE;
   }

   ShowWindow(m_hwnd, nCmdShow);
   UpdateWindow(m_hwnd);

   return TRUE;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE: Processes messages for the main window.
//
//  WM_COMMAND  - process the application menu
//  WM_PAINT    - Paint the main window
//  WM_DESTROY  - post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    HDC hdc = GetDC(hWnd);
    PAINTSTRUCT ps;
    AppState* appstate = reinterpret_cast<AppState*>(GetWindowLongPtr(hWnd, GWLP_USERDATA));

    switch (message)
    {
    case WM_TIMER:
    {
        switch (wParam)
        {
        case ID_REDRAW_CLOCK:
            InvalidateRect(hWnd, NULL, TRUE);
        
        default:
            break;
        }
    }
    case WM_COMMAND:
        {
            int wmId = LOWORD(wParam);
            
            // Parse the menu selections:
            switch (wmId)
            {
            case IDM_ABOUT:
                DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
                break;
            case IDM_EXIT:
                DestroyWindow(hWnd);
                break;
            case ID_DEMO1:
                appstate->demo1();
                break;
            case ID_DEMO2:
                appstate->demo2();
                break;
            case ID_DEMO3:
                appstate->demo3();
                break;
            case ID_DEMO4:
                appstate->demo4();
                break;

            case ID_CATCH :
            {
                appstate->on_catch();
                break;
            }

            case ID_SCRIPT_START:
            {
                appstate->start_script();
            }
            break;

            case ID_SCRIPT_STOP:
            {
                appstate->stop_script();
            }
            break;

            case ID_TRAJECTORY_DEL:
            {
                LRESULT result = ListBox_GetCurSel(appstate->list_box);
                if (result != LB_ERR)
                {
                    appstate->list_box_trajectory.erase(appstate->list_box_trajectory.begin() + result);
                    ListBox_DeleteString(appstate->list_box, result);
                }
                else
                {
                    if (!appstate->list_box_trajectory.empty())
                    {
                        appstate->list_box_trajectory.erase(appstate->list_box_trajectory.end() - 1);
                        ListBox_DeleteString(appstate->list_box, appstate->list_box_trajectory.size());
                    }
                    
                }
                break;
            }
            case ID_TRAJECTORY_CATCH:
            {
                LRESULT result = ListBox_GetCurSel(appstate->list_box);
                if (result != LB_ERR)
                {
                    appstate->list_box_trajectory[result].robotCommand = rc_catch;
                    ListBox_DeleteString(appstate->list_box, result);
                    TCHAR text[25];
                    appstate->print(appstate->list_box_trajectory[result], text);
                    ListBox_InsertString(appstate->list_box, result, text);
                }
                else
                {
                    if (!appstate->list_box_trajectory.empty())
                    {
                        result = appstate->list_box_trajectory.size() - 1;
                        appstate->list_box_trajectory[result].robotCommand = rc_catch;
                        ListBox_DeleteString(appstate->list_box, result);
                        TCHAR text[25];
                        appstate->print(appstate->list_box_trajectory[result], text);
                        ListBox_InsertString(appstate->list_box, result, text);
                    }
                }
                break;
            }

            case ID_TRAJECTORY_STOP:
            {
                LRESULT result = ListBox_GetCurSel(appstate->list_box);
                if (result != LB_ERR)
                {
                    appstate->list_box_trajectory[result].robotCommand = rc_stop;
                    ListBox_DeleteString(appstate->list_box, result);
                    TCHAR text[25];
                    appstate->print(appstate->list_box_trajectory[result], text);
                    ListBox_InsertString(appstate->list_box, result, text);
                }
                else
                {
                    if (!appstate->list_box_trajectory.empty())
                    {
                        result = appstate->list_box_trajectory.size() - 1;
                        appstate->list_box_trajectory[result].robotCommand = rc_stop;
                        ListBox_DeleteString(appstate->list_box, result);
                        TCHAR text[25];
                        appstate->print(appstate->list_box_trajectory[result], text);
                        ListBox_InsertString(appstate->list_box, result, text);
                    }
                }
                break;
            }

            case ID_TRAJECTORY_REL:
            {
                LRESULT result = ListBox_GetCurSel(appstate->list_box);
                if (result != LB_ERR)
                {
                    appstate->list_box_trajectory[result].robotCommand = rc_release;
                    ListBox_DeleteString(appstate->list_box, result);
                    TCHAR text[25];
                    appstate->print(appstate->list_box_trajectory[result], text);
                    ListBox_InsertString(appstate->list_box, result, text);
                }
                else
                {
                    if (!appstate->list_box_trajectory.empty())
                    {
                        result = appstate->list_box_trajectory.size() - 1;
                        appstate->list_box_trajectory[result].robotCommand = rc_release;
                        ListBox_DeleteString(appstate->list_box, result);
                        TCHAR text[25];
                        appstate->print(appstate->list_box_trajectory[result], text);
                        ListBox_InsertString(appstate->list_box, result, text);
                    }
                }
                break;
            }

            case ID_TRAJECTORY_LOAD:
            {
                appstate->SetRobotTrajectory(appstate->list_box_trajectory);
            }

            default:
                return DefWindowProc(hWnd, message, wParam, lParam);
            }
        }
        break;
    case WM_PAINT:
        {
            
            hdc = BeginPaint(hWnd, &ps);
            HDC memDC = CreateCompatibleDC(hdc);
            RECT rcClientRect;
            GetClientRect(hWnd, &rcClientRect);
            HBITMAP bmp = CreateCompatibleBitmap(hdc, rcClientRect.right - rcClientRect.left, rcClientRect.bottom - rcClientRect.top);
            HBITMAP oldBmp = (HBITMAP)SelectObject(memDC, bmp);
            FillRect(memDC, &ps.rcPaint, (HBRUSH)(COLOR_MENU+ 1));
            myOnPaint(memDC, appstate);
            BitBlt(hdc, 0, 0, rcClientRect.right - rcClientRect.left, rcClientRect.bottom - rcClientRect.top, memDC, 0, 0, SRCCOPY);
            SelectObject(memDC, oldBmp);
            DeleteObject(bmp);
            DeleteObject(memDC);
            EndPaint(hWnd, &ps);
        }
        break;
    case WM_ERASEBKGND:
        return 1L;

    case WM_DESTROY:
        PostQuitMessage(0);
        end = TRUE;
        break;

    case WM_QUIT:
        end = TRUE;
        DefWindowProc(hWnd, message, wParam, lParam);
        break;

    case WM_NCCREATE:
    {
        LONG_PTR lptr = reinterpret_cast<LONG_PTR>(reinterpret_cast<CREATESTRUCT*>(lParam)->lpCreateParams);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, lptr);
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    case WM_HSCROLL:
    {
        if (LOWORD(wParam) == SB_THUMBPOSITION || LOWORD(wParam) == TB_THUMBTRACK)
        {
            if (GetWindowLongPtr((HWND)lParam, GWLP_ID) == ID_ROBOT1)
            {
                appstate->SetRobotArm1(HIWORD(wParam));
            }
            else if (GetWindowLongPtr((HWND)lParam, GWLP_ID) == ID_ROBOT2)
            {
                appstate->SetRobotArm2(HIWORD(wParam));
            }
        }
        break;
    }
    case WM_LBUTTONDOWN:
    {
        Point p;
        p.X = LOWORD(lParam);
        p.Y = HIWORD(lParam);
        appstate->SetRobotPosition(p);
        break;
    }
    case WM_RBUTTONUP:
    {
        Point p;
        p.X = LOWORD(lParam);
        p.Y = HIWORD(lParam);
        TCHAR text[50];
        appstate->list_box_trajectory.push_back({ p, rc_do_nothing });
        appstate->print({ p, rc_do_nothing }, text);
        SendMessage(appstate->list_box, LB_ADDSTRING, 0, (LPARAM)text);
        break;
    }
    case WM_SIZING:
    {
        appstate->pause_simulation = true;
        break;
    }
    case WM_SIZE:
    {
        appstate->pause_simulation = false;
        break;
    }
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG:
        return (INT_PTR)TRUE;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}

void myOnPaint(HDC& hdc, AppState* appstate)
{
    Graphics graphics(hdc);
    graphics.SetSmoothingMode(SmoothingModeAntiAlias);
    appstate->draw(&graphics);
}