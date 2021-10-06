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
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <sstream>
#include <fstream>
#include <algorithm>
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
    PointF gravity{ 0, 1e-5 };

    std::vector<Triangle> triangles;
    bool is_robot = false, run_script = false;
    Robot* robot = nullptr;
    int stage;
    std::vector<RobotPosition> predefined_trajectory;

public:
    //pens and brushes in basic colors
    std::vector<Pen*> pens;
    SolidBrush redBrush, greenBrush, blueBrush, whiteBrush, blackBrush;
    Pen redPen, greenPen, bluePen, whitePen;

    //rects
    Rect client_rect;
    //animation rect
    RectF ar;
    HWND list_box;
    
    void print(RobotPosition rp, TCHAR* text)
    {
        TCHAR text1[27], text2[12];
        StringCbPrintf(text1, ARRAYSIZE(text1), TEXT("(%.0f, %.0f) "), rp.pos[0], rp.pos[1]);
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
        redPen(&redBrush, 2), greenPen(&greenBrush, 2), bluePen(&blueBrush, 2), whitePen(&whiteBrush, 2),
        pens{ &redPen, &greenPen, &bluePen },
        //initialize rects
        client_rect(0, 0, 750, 540), ar(10, 10, 480, 370),
        list_box_trajectory(), triangles()
    {
        for (auto & p : pens)
        {
            p->SetLineCap(LineCapRound, LineCapRound, DashCapRound);
        }
        auto logger = spdlog::basic_logger_mt("basic_logger", "logs/basic-log.txt", true);
        
        
        this->predefined_trajectory = { 
            { {60, 60}, rc_stop}, { { 50, 360 }, rc_catch }, { { 200, 200 }, rc_do_nothing },
            { { 400, 360 }, rc_do_nothing}, { { 400, 360 }, rc_release }, { {200, 200}, rc_do_nothing },
            { {250, 360}, rc_catch }, { { 250, 200 }, rc_do_nothing }, { { 350, 360 }, rc_do_nothing},
            { { 350, 360 }, rc_release }, { {200, 200}, rc_do_nothing }, { { 150, 340 }, rc_catch },
            { { 200, 200 }, rc_do_nothing }, { { 375, 280 }, rc_do_nothing }, { { 375, 305 }, rc_release }
        };
       
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
    }

    void demo4()
    {
        triangles.erase(triangles.begin(), triangles.end());
        Triangle::gravity = gravity;
        triangles.push_back(Triangle(PointF(50, 360), 25, 0, PointF(0.0, 0), 0));
        triangles.push_back(Triangle(PointF(150, 360), 50, 0, PointF(0.0, 0), 0));
        triangles.push_back(Triangle(PointF(250, 360), 25, 0, PointF(0.0, 0), 0));
        
        is_robot = true;
        robot = new Robot(triangles);
        robot->set_tPosition(Vector2f{ 200, 200 });
        robot->set_postion({ 200, 200 });
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

    inline void SetRobotArmOmega1(float val)
    {
        if (is_robot)
            robot->omega1 = val;
    }
    
    inline void SetRobotArmOmega2(float val)
    {
        if (is_robot)
            robot->omega2 = val;
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
               /* for (int i = 0; i < triangles.size(); i++)
                {
                    robot->catch_triangle(&triangles[i]);
                }*/
                robot->catch_triangle();
            }
            else
            {
                robot->catched_triangle = nullptr;
            }
           
        }
    }
    
    void start_script()
    {
        this->SetRobotTrajectory(this->predefined_trajectory);

    }

    void stop_script()
    {
        this->robot->following_trajectory = false;
    }

    inline void follow_line(PointF start, PointF dest, float & t1_ms, float t2_ms, int & stage, float dt)
    {
       
    }

    void saveFile() {
        HRESULT hr;
        IFileSaveDialog* pFileSave;
        IShellItem* pItem;
        TCHAR* pFileName;

        hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
        hr = CoCreateInstance(CLSID_FileSaveDialog, nullptr, CLSCTX_ALL, IID_IFileSaveDialog, reinterpret_cast<void**>(&pFileSave));
        COMDLG_FILTERSPEC fileTypes[] = { {TEXT("Trajektoria"), L"*.trajectory"} };
        hr = pFileSave->SetFileTypes(1, fileTypes);
        hr = pFileSave->SetTitle(TEXT("Zapisz trajektoriê"));
        hr = pFileSave->SetFileName(TEXT("1.trajectory"));
        hr = pFileSave->Show(m_hwnd);
        hr = pFileSave->GetResult(&pItem);
        if (hr != S_OK)
            return;
        hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pFileName);

        if (pFileName != nullptr)
        {
            std::ofstream output_file_stream;
            output_file_stream.open(pFileName, std::ios_base::out | std::ios_base::binary);
            std::ostream_iterator<RobotPosition> output_iterator(output_file_stream);
            copy(list_box_trajectory.begin(), list_box_trajectory.end(), output_iterator);
        }

        CoTaskMemFree(pFileName);
        pItem->Release();
        pFileSave->Release();
        CoUninitialize();
    }

    void loadFile()
    {
        HRESULT hr;
        IFileOpenDialog* pFileOpen;
        IShellItem* pItem;
        TCHAR* filename;

        hr = CoInitializeEx(nullptr, COINIT_DISABLE_OLE1DDE | COINIT_APARTMENTTHREADED);
        hr = CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_ALL, IID_IFileOpenDialog,
            reinterpret_cast<void**>(&pFileOpen));
        COMDLG_FILTERSPEC fileTypes[] = { {TEXT("Trajektoria"), L"*.trajectory"} };
        hr = pFileOpen->SetFileTypes(1, fileTypes);
        hr = pFileOpen->SetTitle(TEXT("Otwórz trajektoriê"));
        hr = pFileOpen->SetFileName(TEXT("1.trajectory"));
        hr = pFileOpen->Show(m_hwnd);
        hr = pFileOpen->GetResult(&pItem);
        if (hr != S_OK)
            return;

        hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &filename);
        if (filename)
        {
            std::ifstream input_file_stream;
            input_file_stream.open(filename, std::ios::out | std::ios::binary);
            RobotPosition temp_robot_position;
            while (input_file_stream >> temp_robot_position)
            {
                list_box_trajectory.push_back(temp_robot_position);
                TCHAR text[50];
                print(temp_robot_position, text);
                SendMessage(list_box, LB_ADDSTRING, NULL, (LPARAM)text);
            }
        }

        CoTaskMemFree(filename);
        pItem->Release();
        pFileOpen->Release();
        CoUninitialize();
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
    std::chrono::milliseconds update_interval(10);
    do
    {
        
        
        std::chrono::milliseconds dt(std::chrono::duration_cast<std::chrono::milliseconds>(sc.now() - one_loop_start));
       
        if (dt > update_interval)
        {
            std::chrono::steady_clock::time_point temp = sc.now();
            appState->update(dt.count());
            if (dt.count() < 50)
                appState->update(dt.count());
            else
                appState->update(50);
            spdlog::get("basic_logger")->info("dt.count() = {}", dt.count());
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


   DWORD ws_style = WS_CLIPCHILDREN | WS_OVERLAPPED | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SYSMENU | WS_CAPTION | WS_VISIBLE | WS_BORDER ;
   Rect window_rect(appstate->client_rect);
   AdjustWindowRectEx(reinterpret_cast<LPRECT>(&window_rect), ws_style, TRUE, WS_EX_OVERLAPPEDWINDOW);


   m_hwnd = CreateWindowEx(WS_EX_OVERLAPPEDWINDOW, szWindowClass, szTitle, ws_style,
      CW_USEDEFAULT, CW_USEDEFAULT, window_rect.GetRight() - window_rect.GetLeft() + 10, window_rect.GetBottom() - window_rect.GetTop(),
       nullptr, nullptr, hInstance, reinterpret_cast<void*>(appstate));

   CreateWindow(WC_BUTTON, TEXT("demo 1"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->client_rect.GetLeft(), appstate->ar.GetBottom() + 10, BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_DEMO1, hInstance, NULL);
   
   CreateWindow(WC_BUTTON, TEXT("demo 2"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->client_rect.GetLeft() + BUTTON_WIDTH, appstate->ar.GetBottom() + 10, BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_DEMO2, hInstance, NULL);
   
   CreateWindow(WC_BUTTON, TEXT("demo 3"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->client_rect.GetLeft() + BUTTON_WIDTH*2, appstate->ar.GetBottom() + 10, BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_DEMO3, hInstance, NULL);
   
   CreateWindow(WC_BUTTON, TEXT("demo 4"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->client_rect.GetLeft() + BUTTON_WIDTH*3, appstate->ar.GetBottom() + 10, BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_DEMO4, hInstance, NULL);

   HWND hwnd = CreateWindowEx(NULL, TRACKBAR_CLASS, TEXT("robot control 1"), WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
       appstate->client_rect.GetLeft(), appstate->ar.GetBottom() + 10 + BUTTON_HEIGHT, 2 * BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_ROBOT1, hInstance, NULL);

   SendMessage(hwnd, TBM_SETRANGE, TRUE, MAKELPARAM(20, 120));
   SendMessage(hwnd, TBM_SETPOS, TRUE, 60);
   SendMessage(hwnd, TBM_SETTICFREQ, 10, 0);

   hwnd = CreateWindowEx(NULL, TRACKBAR_CLASS, TEXT("robot control 2"), WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
       appstate->client_rect.GetLeft() + 2*BUTTON_WIDTH, appstate->ar.GetBottom() + 10 + BUTTON_HEIGHT, 2 * BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_ROBOT2, hInstance, NULL);

   SendMessage(hwnd, TBM_SETRANGE, TRUE, MAKELPARAM(20, 120));
   SendMessage(hwnd, TBM_SETPOS, TRUE, 60);
   SendMessage(hwnd, TBM_SETTICFREQ, 10, 0);

   CreateWindow(WC_BUTTON, TEXT("catch"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->client_rect.GetLeft() + BUTTON_WIDTH * 4, appstate->ar.GetBottom() + 10 + BUTTON_HEIGHT, BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_CATCH, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("start script"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->client_rect.GetLeft() + BUTTON_WIDTH * 3, appstate->ar.GetBottom() + 10 + BUTTON_HEIGHT*2, BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_SCRIPT_START, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("stop script"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->client_rect.GetLeft() + BUTTON_WIDTH * 4, appstate->ar.GetBottom() + 10 + BUTTON_HEIGHT*2, BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_SCRIPT_STOP, hInstance, NULL);

   HWND list_box_hwnd = CreateWindowEx(WS_EX_CLIENTEDGE, WC_LISTBOX, TEXT("list view"), WS_CHILD | LBS_HASSTRINGS | WS_VISIBLE | WS_BORDER,
       appstate->ar.GetRight()+20, appstate->ar.GetTop(), BUTTON_WIDTH * 2, appstate->ar.GetBottom() - appstate->ar.GetTop(),
       m_hwnd, NULL, hInstance, NULL);
   appstate->list_box = list_box_hwnd;

   CreateWindow(WC_BUTTON, TEXT("usuñ"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->ar.GetRight()+20, appstate->ar.GetBottom() + 0 + 0 * BUTTON_HEIGHT, BUTTON_WIDTH * 0.5, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_TRAJECTORY_DEL, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("catch"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->ar.GetRight() + 0.5*BUTTON_WIDTH + 20, appstate->ar.GetBottom() + 0 + 0 * BUTTON_HEIGHT, BUTTON_WIDTH * 0.5, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_TRAJECTORY_CATCH, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("drop"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->ar.GetRight() + BUTTON_WIDTH + 20, appstate->ar.GetBottom() + 0 + 0 * BUTTON_HEIGHT, BUTTON_WIDTH * 0.5, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_TRAJECTORY_REL, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("stop"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->ar.GetRight() + 1.5*BUTTON_WIDTH + 20, appstate->ar.GetBottom() + 0 + 0 * BUTTON_HEIGHT, BUTTON_WIDTH * 0.5, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_TRAJECTORY_STOP, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("follow"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->ar.GetRight() + 0 * BUTTON_WIDTH + 20, appstate->ar.GetBottom() + 1 * BUTTON_HEIGHT, BUTTON_WIDTH * 1, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_TRAJECTORY_LOAD, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("Save file"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->ar.GetRight() + 20, appstate->ar.GetBottom() +2 * BUTTON_HEIGHT, BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_TRAJECTORY_SAVE, hInstance, NULL);

   CreateWindow(WC_BUTTON, TEXT("Load file"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
       appstate->ar.GetRight() + BUTTON_WIDTH + 20, appstate->ar.GetBottom() +2 * BUTTON_HEIGHT, BUTTON_WIDTH, BUTTON_HEIGHT,
       m_hwnd, (HMENU)ID_TRAJECTORY_READ_FROM_FILE, hInstance, NULL);



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
                    if (appstate->list_box_trajectory.size() != 0)
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
                if (result == LB_ERR == appstate->list_box_trajectory.size() > 0)
                    result = appstate->list_box_trajectory.size() - 1;
                appstate->list_box_trajectory[result].robotCommand = rc_catch;
                ListBox_DeleteString(appstate->list_box, result);
                TCHAR text[25];
                appstate->print(appstate->list_box_trajectory[result], text);
                ListBox_InsertString(appstate->list_box, result, text);
                break;
            }

            case ID_TRAJECTORY_STOP:
            {
                LRESULT result = ListBox_GetCurSel(appstate->list_box);
                if (result == LB_ERR == appstate->list_box_trajectory.size() > 0)
                    result = appstate->list_box_trajectory.size() - 1;
                appstate->list_box_trajectory[result].robotCommand = rc_stop;
                ListBox_DeleteString(appstate->list_box, result);
                TCHAR text[25];
                appstate->print(appstate->list_box_trajectory[result], text);
                ListBox_InsertString(appstate->list_box, result, text);
                break;
            }

            case ID_TRAJECTORY_REL:
            {
                LRESULT result = ListBox_GetCurSel(appstate->list_box);
                if (result == LB_ERR == appstate->list_box_trajectory.size() > 0)
                    result = appstate->list_box_trajectory.size() - 1;
                appstate->list_box_trajectory[result].robotCommand = rc_release;
                ListBox_DeleteString(appstate->list_box, result);
                TCHAR text[25];
                appstate->print(appstate->list_box_trajectory[result], text);
                ListBox_InsertString(appstate->list_box, result, text);
                break;
            }

            case ID_TRAJECTORY_LOAD:
            {
                appstate->SetRobotTrajectory(appstate->list_box_trajectory);
                break;
            }

            case ID_TRAJECTORY_SAVE:
            {
                appstate->saveFile();
                break;
            }

            case ID_TRAJECTORY_READ_FROM_FILE:
            {
                appstate->loadFile();
                break;
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
                appstate->SetRobotArmOmega1(HIWORD(wParam) / 1e4);
            }
            else if (GetWindowLongPtr((HWND)lParam, GWLP_ID) == ID_ROBOT2)
            {
                appstate->SetRobotArmOmega2(HIWORD(wParam) / 1e4);
            }
        }
        break;
    }
    case WM_LBUTTONDOWN:
    {
        Point p;
        p.X = LOWORD(lParam);
        p.Y = HIWORD(lParam);
        if (appstate->ar.Contains({ (REAL)p.X, (REAL)p.Y }))
            appstate->SetRobotTrajectory({ {{p.X, p.Y}, rc_stop} });
        break;
    }
    case WM_RBUTTONUP:
    {
        Point p;
        p.X = LOWORD(lParam);
        p.Y = HIWORD(lParam);
        if (appstate->ar.Contains({ (REAL)p.X, (REAL)p.Y }))
        {
            TCHAR text[50];
            appstate->list_box_trajectory.push_back({ {p.X, p.Y}, rc_do_nothing });
            appstate->print({ {p.X, p.Y}, rc_do_nothing }, text);
            SendMessage(appstate->list_box, LB_ADDSTRING, 0, (LPARAM)text);
        }
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

