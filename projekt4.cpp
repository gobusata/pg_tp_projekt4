// projekt4.cpp : Defines the entry point for the application.
//

#include <windows.h>
#include <gdiplus.h>
#include <vector>
#include <chrono>
#include <ShObjIdl.h>
using namespace Gdiplus;

#include "framework.h"
#include "projekt4.h"
#include "Triangle.h"
#include "Robot.h"

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
const int BUTTON_WIDTH = 100, BUTTON_HEIGHT = 30;


HWND m_hwnd;
BOOL end = FALSE;
//appplication state
class AppState
{
private:   
    //gravity
    PointF gravity{ 0, 2e-5 };
    std::vector<Triangle> triangles;
    bool is_robot = false, run_script = false;
    Robot* robot = nullptr;
    int stage;

public:
    //pens and brushes in basic colors
    //Brush* brushes[5];
    std::vector<Pen*> pens;
    SolidBrush redBrush, greenBrush, blueBrush, whiteBrush;
    Pen redPen, greenPen, bluePen, whitePen;

    //rects
    Rect client_rect;
    //animation rect
    RectF ar;

 
    AppState():
        //initialize pens and brushes
        redBrush(Color(215, 20, 20)), greenBrush(Color(20, 215, 20)), blueBrush(Color(20, 20, 215)), whiteBrush(Color(200, 200, 200)),
        redPen(&redBrush, 2), greenPen(&greenBrush, 2), bluePen(&blueBrush, 2), whitePen(&whiteBrush, 2),
        pens{ &redPen, &greenPen, &bluePen },
        //initialize rects
        client_rect(0, 0, 500, 530), ar(10, 10, 480, 370)
    {
        for (auto & p : pens)
        {
            p->SetLineCap(LineCapRound, LineCapRound, DashCapRound);
        }

    }

    void draw(Graphics* graphics)
    {
        
        graphics->DrawRectangle(&bluePen, ar);
        graphics->FillRectangle(&whiteBrush, ar);
        for (auto tri : triangles)
        {
            tri.draw(graphics, &greenPen);
        }

        if (is_robot) robot->draw(graphics);

    }

    void update(REAL dt)
    {
        
        for (std::vector<Triangle>::iterator tri = triangles.begin(); tri != triangles.end(); tri++)
        {
            tri->update(dt);
            tri->collision_with_wall(&ar);
            for (std::vector<Triangle>::iterator tri2 = tri + 1; tri2 != triangles.end(); tri2++)
                tri->collision_with_figure(*tri2, dt);
        }
        if (is_robot)
        {
            robot->update(dt);
            if (run_script) script(dt);
        }
       
    }

    void demo1()
    {
        Triangle::gravity = gravity;
        is_robot = false;
        triangles.erase(triangles.begin(), triangles.end());
        triangles.push_back(Triangle(PointF(100, 100), 30, 120, { 0, 0 }, 0.0002));
        triangles.push_back(Triangle(PointF(170, 120), 30, 120, { 0, 0 }, 0.0004));
        triangles.push_back(Triangle(PointF(240, 150), 30, 120, { 0, 0 }, 0.0006));
        triangles.push_back(Triangle(PointF(310, 170), 30, 120, { 0, 0 }, 0.0008));

    }

    void demo2()
    {
        Triangle::gravity = gravity;
        is_robot = false;
        triangles.erase(triangles.begin(), triangles.end());
        triangles.push_back(Triangle(PointF(110, 250), 100, 170, { 0, 0 }, 0));
        triangles.push_back(Triangle(PointF(200, 100), 30, 0, { 0, 0 }, 0));
    }

    void demo3()
    {
        triangles.erase(triangles.begin(), triangles.end());
        Triangle::gravity = gravity;
        is_robot = false;
        triangles.push_back(Triangle(PointF(250, 360), 50, 0, PointF(0.0, 0), 0));
        triangles.push_back(Triangle(PointF(265, 200), 50, 0, PointF(0.0, 0), 0));
        triangles.push_back(Triangle(PointF(233, 280), 50, 0, PointF(0.0, 0), 0));
        /*triangles.push_back(Triangle(PointF(180, 200), 25, 0, PointF(0.0, 0), 0));
        triangles.push_back(Triangle(PointF(230, 280), 50, 180, PointF(0.0, 0), 0));
        triangles.push_back(Triangle(PointF(280, 200), 25, 0, PointF(0.0, 0), 0));*/

    }

    void demo4()
    {
        triangles.erase(triangles.begin(), triangles.end());
        Triangle::gravity = gravity;
        triangles.push_back(Triangle(PointF(50, 360), 25, 0, PointF(0.0, 0), 0));
        triangles.push_back(Triangle(PointF(150, 360), 50, 0, PointF(0.0, 0), 0));
        triangles.push_back(Triangle(PointF(250, 360), 25, 0, PointF(0.0, 0), 0));
        
        is_robot = true;
        robot = new Robot();
        stage = 0;
        robot->set_postion({ 60, 60 });
    }

    inline void SetRobotArm1(REAL a)
    {
        if (is_robot)
        {
            robot->set_arms(a - 90, robot->angle2_deg);
        }
    }

    inline void SetRobotArm2(REAL a)
    {
        if (is_robot)
        {
            robot->set_arms(robot->angle1_deg, a - 180);
        }
    }

    void on_catch()
    {
        if (is_robot)
        {
            if (robot->catched_triangle == nullptr)
            {
                for (int i = 0; i < triangles.size(); i++)
                {
                    robot->catch_triangle(&triangles[i]);
                }
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

    // TODO: Place code here.

    // Initialize global strings
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_PROJEKT4, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    ULONG_PTR token;
    GdiplusStartupInput gdiinput;
    GdiplusStartupOutput gdioutput;
    GdiplusStartup(&token, &gdiinput, &gdioutput);


    AppState *appState = new AppState;
    //appState->demo4();


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
    std::chrono::duration<REAL, std::milli> update_interval(1e-1);
    do
    {
        
        
        std::chrono::duration<REAL, std::milli> dt(sc.now() - one_loop_start);
       
        if (dt > update_interval)
        {
            std::chrono::steady_clock::time_point temp = sc.now();
            appState->update(dt.count());
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


    //while (GetMessage(&msg, nullptr, 0, 0))
    //{
    //    if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
    //    {
    //        TranslateMessage(&msg);
    //        DispatchMessage(&msg);
    //    }
    //}
    

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


   DWORD ws_style = WS_CLIPCHILDREN | WS_OVERLAPPED | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SYSMENU | WS_CAPTION | WS_VISIBLE | WS_BORDER ;
   Rect window_rect(appstate->client_rect);
   AdjustWindowRectEx(reinterpret_cast<LPRECT>(&window_rect), ws_style, TRUE, WS_EX_OVERLAPPEDWINDOW);


   m_hwnd = CreateWindowEx(WS_EX_OVERLAPPEDWINDOW, szWindowClass, szTitle, ws_style,
      CW_USEDEFAULT, CW_USEDEFAULT, window_rect.GetRight() - window_rect.GetLeft() + 10, window_rect.GetBottom() - window_rect.GetTop(),
       nullptr, nullptr, hInstance, reinterpret_cast<void*>(appstate));

   CreateWindow(WC_BUTTON, TEXT("demo 1"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetLeft(), appstate->ar.GetBottom(), BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_DEMO1, hInstance, NULL);
   
   CreateWindow(WC_BUTTON, TEXT("demo 2"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetLeft() + BUTTON_WIDTH, appstate->ar.GetBottom(), BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_DEMO2, hInstance, NULL);
   
   CreateWindow(WC_BUTTON, TEXT("demo 3"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetLeft() + BUTTON_WIDTH*2, appstate->ar.GetBottom(), BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_DEMO3, hInstance, NULL);
   
   CreateWindow(WC_BUTTON, TEXT("demo 4"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetLeft() + BUTTON_WIDTH*3, appstate->ar.GetBottom(), BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_DEMO4, hInstance, NULL);

   HWND hwnd = CreateWindowEx(NULL, TRACKBAR_CLASS, TEXT("robot control 1"), WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
       appstate->client_rect.GetLeft(), appstate->ar.GetBottom() + BUTTON_HEIGHT, 2 * BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_ROBOT1, hInstance, NULL);

   SendMessage(hwnd, TBM_SETRANGE, TRUE, MAKELPARAM(0, 180));
   SendMessage(hwnd, TBM_SETPOS, TRUE, 45);
   SendMessage(hwnd, TBM_SETTICFREQ, 10, 0);

   hwnd = CreateWindowEx(NULL, TRACKBAR_CLASS, TEXT("robot control 2"), WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
       appstate->client_rect.GetLeft() + 2*BUTTON_WIDTH, appstate->ar.GetBottom() + BUTTON_HEIGHT, 2 * BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_ROBOT2, hInstance, NULL);

   SendMessage(hwnd, TBM_SETRANGE, TRUE, MAKELPARAM(0, 360));
   SendMessage(hwnd, TBM_SETPOS, TRUE, 315);
   SendMessage(hwnd, TBM_SETTICFREQ, 10, 0);

   CreateWindow(WC_BUTTON, TEXT("catch"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetLeft() + BUTTON_WIDTH * 4, appstate->ar.GetBottom() + BUTTON_HEIGHT, BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_CATCH, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("start script"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetLeft() + BUTTON_WIDTH * 3, appstate->ar.GetBottom() + BUTTON_HEIGHT*2, BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_SCRIPT_START, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("stop script"), WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
       appstate->client_rect.GetLeft() + BUTTON_WIDTH * 4, appstate->ar.GetBottom() + BUTTON_HEIGHT*2, BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_SCRIPT_STOP, hInstance, NULL);

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
    appstate->draw(&graphics);
}