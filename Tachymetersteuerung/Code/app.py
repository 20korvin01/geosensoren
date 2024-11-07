import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from threading import Thread
import webbrowser
import dash_bootstrap_components as dbc
from dash import Dash, html, dcc, Input, Output, State, callback_context, no_update
import utils as utl


# Function to open the Dash app in the default web browser
def open_app():
    webbrowser.open("http://127.0.0.1:8050/")


def layout_dash():
    # defining styles
    COM_label = {
        "color": "silver",
        "margin-bottom": "0px",
        "font-size": "30px",
        "font-weight": "bold",
        "margin-right": "10px",
    }
    COM_input = {
        "color": "gray",
        "height": "50px",
        "font-weight": "bold",
        "font-size": "30px",
        "text-align": "center",
        "backgroundColor": "silver",
        "border": "1px solid gray",
    }
    alerts = {
        "padding-top": "20px",
        "color": "silver",
        "background": "#2b70e0",
        "text-align": "center",
    }
    output = {
        "padding": "0px",
        "color": "gray",
        "width": "310px",
        "height": "50px",
        "text-align": "center",
        "background": "silver",
        "border": "1px solid gray",
        "border-radius": "5px",
        "font-family": "courier new",
        "font-size": "15px",
        "font-weight": "bold",
        "display": "flex",
        "justify-content": "center",
        "align-items": "center",
    }
    output_label = {
        "color": "silver",
        "height": "50px",
        "text-align": "center",
        "background": "#2b70e0",
        "font-size": "30px",
        "font-weight": "bold",
        "border-radius": "5px",
        "margin-bottom": "-10px",
    }
    # tooltips for more information
    tooltips = html.Div(
        [
            dbc.Tooltip("Laser", target="laser_btn", placement="left"),
            dbc.Tooltip("Info", target="info_btn", placement="bottom"),
            dbc.Tooltip("Messanordnung", target="plan_btn", placement="bottom"),
        ]
    )
    # modals
    modals = html.Div(
        [
            dbc.Modal(
                [
                    dbc.ModalHeader(dbc.ModalTitle("Messanordnung")),
                    dbc.ModalBody(
                        html.Div(
                            [
                                html.Img(
                                    src="assets/messanordnung.png",
                                    style={
                                        "max-width": "100%",
                                        "max-height": "100%",
                                        "margin": "auto",
                                    },
                                )
                            ],
                            style={
                                "display": "flex",
                                "justify-content": "center",
                                "align-items": "center",
                                "height": "100%",
                                "width": "100%",
                            },
                        )
                    ),
                ],
                id="plan_modal",
                fullscreen=True,
                is_open=False,
            ),
            dbc.Modal(
                dbc.ModalHeader(
                    dbc.Alert(
                        [
                            html.Div(
                                html.Img(src="assets/error_sign.svg"),
                                style={"margin-right": "30px"},
                            ),
                            "Please enter a valid COM port!",
                        ],
                        className="d-flex align-items-center",
                        style={
                            "padding-top": "20px",
                            "margin-top": "12px",
                            "margin-left": "-6px",
                            "width": "500px",
                            "color": "silver",
                            "background": "transparent",
                        },
                    ),
                    style={
                        "background": "rgba(255, 0, 0, 0.3)",
                        "border-radius": 5,
                        "border": "1px solid #FF0000",
                        "height": "70px",
                    },
                ),
                id="laser_warning",
                is_open=False,
            ),
            dbc.Modal(
                dbc.ModalHeader(
                    dbc.Alert(
                        [
                            html.Div(
                                html.Img(src="assets/error_sign.svg"),
                                style={"margin-right": "30px"},
                            ),
                            "Please enter a valid COM port!",
                        ],
                        className="d-flex align-items-center",
                        style={
                            "padding-top": "20px",
                            "margin-top": "12px",
                            "margin-left": "-6px",
                            "width": "500px",
                            "color": "silver",
                            "background": "transparent",
                        },
                    ),
                    style={
                        "background": "rgba(255, 0, 0, 0.3)",
                        "border-radius": 5,
                        "border": "1px solid #FF0000",
                        "height": "70px",
                    },
                ),
                id="fun_warning",
                is_open=False,
            ),
            dbc.Modal(
                dbc.ModalHeader(
                    dbc.Alert(
                        [
                            html.Div(
                                html.Img(src="assets/error_sign.svg"),
                                style={"margin-right": "30px"},
                            ),
                            "Please enter a valid COM port!",
                        ],
                        className="d-flex align-items-center",
                        style={
                            "padding-top": "20px",
                            "margin-top": "12px",
                            "margin-left": "-6px",
                            "width": "500px",
                            "color": "silver",
                            "background": "transparent",
                        },
                    ),
                    style={
                        "background": "rgba(255, 0, 0, 0.3)",
                        "border-radius": 5,
                        "border": "1px solid #FF0000",
                        "height": "70px",
                    },
                ),
                id="face_warning",
                is_open=False,
            ),
            dbc.Modal(
                dbc.ModalHeader(
                    dbc.Alert(
                        [
                            html.Div(
                                html.Img(src="assets/error_sign.svg"),
                                style={"margin-right": "30px"},
                            ),
                            "Please enter a valid COM port!",
                        ],
                        className="d-flex align-items-center",
                        style={
                            "padding-top": "20px",
                            "margin-top": "12px",
                            "margin-left": "-6px",
                            "width": "500px",
                            "color": "silver",
                            "background": "transparent",
                        },
                    ),
                    style={
                        "background": "rgba(255, 0, 0, 0.3)",
                        "border-radius": 5,
                        "border": "1px solid #FF0000",
                        "height": "70px",
                    },
                ),
                id="date_warning",
                is_open=False,
            ),
            dbc.Modal(
                dbc.ModalHeader(
                    dbc.Alert(
                        [
                            html.Div(
                                html.Img(src="assets/error_sign.svg"),
                                style={"margin-right": "30px"},
                            ),
                            "Please enter valid COM port and file paths!",
                        ],
                        className="d-flex align-items-center",
                        style={
                            "padding-top": "20px",
                            "margin-top": "12px",
                            "margin-left": "-6px",
                            "width": "500px",
                            "color": "silver",
                            "background": "transparent",
                        },
                    ),
                    style={
                        "background": "rgba(255, 0, 0, 0.3)",
                        "border-radius": 5,
                        "border": "1px solid #FF0000",
                        "height": "70px",
                    },
                ),
                id="run_warning",
                is_open=False,
            ),
            dbc.Modal(
                dbc.ModalHeader(
                    dbc.Alert(
                        [
                            html.Div(
                                html.Img(src="assets/error_sign.svg"),
                                style={"margin-right": "30px"},
                            ),
                            "Please enter a valid COM port and point file path!",
                        ],
                        className="d-flex align-items-center",
                        style={
                            "padding-top": "20px",
                            "margin-top": "12px",
                            "margin-left": "-6px",
                            "width": "500px",
                            "color": "silver",
                            "background": "transparent",
                        },
                    ),
                    style={
                        "background": "rgba(255, 0, 0, 0.3)",
                        "border-radius": 5,
                        "border": "1px solid #FF0000",
                        "height": "70px",
                    },
                ),
                id="start_warning",
                is_open=False,
            ),
            dbc.Modal(
                dbc.ModalHeader(
                    dbc.Alert(
                        [
                            html.Div(
                                html.Img(src="assets/done_sign.svg"),
                                style={"margin-right": "30px"},
                            ),
                            "Run successful!",
                        ],
                        className="d-flex align-items-center",
                        style={
                            "padding-top": "20px",
                            "margin-top": "12px",
                            "margin-left": "-6px",
                            "width": "500px",
                            "color": "silver",
                            "background": "transparent",
                        },
                    ),
                    style={
                        "background": "rgba(0, 179, 0, 0.3)",
                        "border-radius": 5,
                        "border": "1px solid #00b300",
                        "height": "70px",
                    },
                ),
                id="run_success",
                is_open=False,
            ),
            dbc.Modal(
                dbc.ModalHeader(
                    dbc.Alert(
                        [
                            html.Div(
                                html.Img(src="assets/done_sign.svg"),
                                style={"margin-right": "30px"},
                            ),
                            "Date and time set successfully!",
                        ],
                        className="d-flex align-items-center",
                        style={
                            "padding-top": "20px",
                            "margin-top": "12px",
                            "margin-left": "-6px",
                            "width": "500px",
                            "color": "silver",
                            "background": "transparent",
                        },
                    ),
                    style={
                        "background": "rgba(0, 179, 0, 0.3)",
                        "border-radius": 5,
                        "border": "1px solid #00b300",
                        "height": "70px",
                    },
                ),
                id="date_success",
                is_open=False,
            ),
            dbc.Modal(
                dbc.ModalHeader(
                    dbc.Alert(
                        [
                            html.Div(
                                html.Img(src="assets/done_sign.svg"),
                                style={"margin-right": "30px"},
                            ),
                            "Face switched successfully!",
                        ],
                        className="d-flex align-items-center",
                        style={
                            "padding-top": "20px",
                            "margin-top": "12px",
                            "margin-left": "-6px",
                            "width": "500px",
                            "color": "silver",
                            "background": "transparent",
                        },
                    ),
                    style={
                        "background": "rgba(0, 179, 0, 0.3)",
                        "border-radius": 5,
                        "border": "1px solid #00b300",
                        "height": "70px",
                    },
                ),
                id="switch_face_success",
                is_open=False,
            ),
            dbc.Modal(
                dbc.ModalHeader(
                    dbc.Alert(
                        [
                            html.Div(
                                html.Img(src="assets/done_sign.svg"),
                                style={"margin-right": "30px"},
                            ),
                            "Finding start point and setting orientation successful!",
                        ],
                        className="d-flex align-items-center",
                        style={
                            "padding-top": "20px",
                            "margin-top": "12px",
                            "margin-left": "-6px",
                            "width": "500px",
                            "color": "silver",
                            "background": "transparent",
                        },
                    ),
                    style={
                        "background": "rgba(0, 179, 0, 0.3)",
                        "border-radius": 5,
                        "border": "1px solid #00b300",
                        "height": "70px",
                    },
                ),
                id="start_success",
                is_open=False,
            ),
            dbc.Modal(
                dbc.ModalHeader(
                    dbc.Alert(
                        [
                            html.Div(
                                html.Img(src="assets/caution_sign.svg"),
                                style={"margin-right": "30px"},
                            ),
                            "Temporarily disabled!",
                        ],
                        className="d-flex align-items-center",
                        style={
                            "padding-top": "20px",
                            "margin-top": "12px",
                            "margin-left": "-6px",
                            "width": "500px",
                            "color": "silver",
                            "background": "transparent",
                        },
                    ),
                    style={
                        "background": "rgba(243, 156, 18, 0.3)",
                        "border-radius": 5,
                        "border": "1px solid #F39C12",
                        "height": "70px",
                    },
                ),
                id="search_caution",
                is_open=False,
            ),
        ]
    )
    spinners = html.Div(
        [
            dbc.Spinner(
                children=[html.Div(id="run_spinner", style={"display": "none"})],
                type=None,
                fullscreen=True,
                fullscreen_style={
                    "opacity": "0.5",
                    "z-index": "10000",
                    "backgroundColor": "transparent",
                },
                spinnerClassName="spinner",
            ),
            dbc.Spinner(
                children=[html.Div(id="fun_spinner", style={"display": "none"})],
                type=None,
                fullscreen=True,
                fullscreen_style={
                    "opacity": "0.5",
                    "z-index": "10000",
                    "backgroundColor": "transparent",
                },
                spinnerClassName="spinner",
            ),
            dbc.Spinner(
                children=[
                    html.Div(id="switch_face_spinner", style={"display": "none"})
                ],
                type=None,
                fullscreen=True,
                fullscreen_style={
                    "opacity": "0.5",
                    "z-index": "10000",
                    "backgroundColor": "transparent",
                },
                spinnerClassName="spinner",
            ),
            dbc.Spinner(
                children=[html.Div(id="start_spinner", style={"display": "none"})],
                type=None,
                fullscreen=True,
                fullscreen_style={
                    "opacity": "0.5",
                    "z-index": "10000",
                    "backgroundColor": "transparent",
                },
                spinnerClassName="spinner",
            ),
        ]
    )
    # info canvas
    info_canvas = html.Div(
        [
            dbc.Offcanvas(
                [
                    html.Div(
                        [
                            # a priori
                            html.H5(
                                "A PRIORI",
                                style={"text-align": "center", "color": "silver"},
                            ),
                            html.Hr(
                                style={
                                    "margin": "auto",
                                    "width": "80%",
                                    "color": "silver",
                                    "margin-bottom": "3px",
                                }
                            ),
                            html.P(
                                "Zunächst sollte die Totalstation im Geolabor auf den Betonpfeiler Nummer 'LP 3' gestellt werden.",
                                style={"color": "gray"},
                            ),
                            html.P(
                                "Anschließend sollte die Totalstation angeschalten und mit dem Laptop verbunden werden.",
                                style={"color": "gray"},
                            ),
                        ],
                        style={
                            "border": "1px solid #3B5A7F",
                            "border-radius": 10,
                            "padding": "10px",
                            "margin-bottom": "16px",
                        },
                    ),
                    html.Div(
                        [
                            # Eingabe
                            html.H5(
                                "EINGABE",
                                style={"text-align": "center", "color": "silver"},
                            ),
                            html.Hr(
                                style={
                                    "margin": "auto",
                                    "width": "80%",
                                    "color": "silver",
                                    "margin-bottom": "3px",
                                }
                            ),
                            html.P(
                                "Der entsprechende COM-Port sollte als Integer eingetragen werden.",
                                style={"color": "gray"},
                            ),
                            html.P(
                                "Für die eigentliche Messung sollte eine Datei (CSV) mit den polaren Messungen (Hz, V, Schrägstrecke) inklusive der Punktnummern als Nullepoche vorliegen; Format: PN,Hz,V,D",
                                style={"color": "gray", "margin-bottom": "0px"},
                            ),
                            html.P(
                                "Beispiel: 'Epoche0_1001-3.csv' für die Punkte 1001-1003, oder 'Epoche0_1001-4.csv' für die Punkte 1001-1004",
                                style={"color": "gray", "margin-top": "0px"},
                            ),
                            html.P(
                                "Des Weiteren sollte auch eine Datei (CSV) als Speicher für die folgenden Messungen angegeben werden; Format: PN,X,Y,Z,Hz,V,D,face,time",
                                style={"color": "gray", "margin-bottom": "0px"},
                            ),
                            html.P(
                                "Beispiel: 'measurements.csv'",
                                style={"color": "gray", "margin-top": "0px"},
                            ),
                            html.P(
                                "Mit den beiden Slidern kann einerseits die Wartezeit zwischen den Epochen und andererseits die Anzahl der Epochen festgelegt werden. Default ist eine Wartezeit von 30 Sekunden und eine Epochenanzahl von 1 eingestellt.",
                                style={"color": "gray"},
                            ),
                        ],
                        style={
                            "border": "1px solid #3B5A7F",
                            "border-radius": 10,
                            "padding": "10px",
                            "margin-bottom": "16px",
                        },
                    ),
                    html.Div(
                        [
                            # Programme
                            html.H5(
                                "PROGRAMME",
                                style={"text-align": "center", "color": "silver"},
                            ),
                            html.Hr(
                                style={
                                    "margin": "auto",
                                    "width": "80%",
                                    "color": "silver",
                                    "margin-bottom": "3px",
                                }
                            ),
                            html.P(
                                "Bevor das Hauptprogramm (RUN PROCESS) gestartet werden kann, sollte zunächst die Nullrichtung gefunden werden. Diese ergibt sich aus der Richtung zum Punkt 1001. Diesen Punkt kann das Programm 'FIND START' selbständig finden.",
                                style={"color": "gray"},
                            ),
                            html.P(
                                "Bei erfolgreicher Durchführung des Programms 'FIND START' sollte die Nullrichtung der Totalstation übergeben worden sein und eine Erfolgsmeldung am Bildschirm sichtbar sein.",
                                style={"color": "gray"},
                            ),
                            html.P(
                                "Nun kann das Hauptprogramm 'RUN PROCESS' gestartet werden. Hierbei werden die Messungen durchgeführt und die Daten in die angegebene Ausgabe-Datei geschrieben.",
                                style={"color": "gray"},
                            ),
                        ],
                        style={
                            "border": "1px solid #3B5A7F",
                            "border-radius": 10,
                            "padding": "10px",
                            "margin-bottom": "16px",
                        },
                    ),
                ],
                id="info_canvas",
                scrollable=False,
                title="Informationen",
                is_open=False,
            )
        ]
    )
    # putting everything together
    return html.Div(
        [
            # tooltips
            tooltips,
            # warning
            modals,
            # spinners
            spinners,
            # info canvas
            info_canvas,
            # header
            html.Br(),
            html.H2(
                "Tachymetersteuerung",
                style={
                    "color": "white",
                    "text-align": "center",
                    "font-weight": "bold",
                    "font-size": "50px",
                },
            ),
            # info button
            html.Div(
                [
                    dbc.ButtonGroup(
                        [
                            dbc.Button(
                                html.Img(src="assets/info_sign.svg"),
                                color="success",
                                outline=True,
                                id="info_btn",
                                style={
                                    "margin-left": "30px",
                                    "margin-top": "-50px",
                                    "float": "left",
                                    "font-size": "30px",
                                },
                            ),
                            dbc.Button(
                                html.Img(src="assets/plan_sign.svg"),
                                color="success",
                                outline=True,
                                id="plan_btn",
                                style={
                                    "margin-top": "-50px",
                                    "float": "left",
                                    "font-size": "30px",
                                },
                            ),
                        ],
                        size="lg",
                        className="me-1",
                    ),
                ]
            ),
            # laser button
            html.Div(
                [
                    dbc.Button(
                        "",
                        color="dark",
                        outline=True,
                        id="laser_btn",
                        style={
                            "margin-right": "30px",
                            "margin-top": "-50px",
                            "float": "right",
                            "font-size": "30px",
                        },
                    ),
                ]
            ),
            html.Br(),
            html.Div(
                [
                    html.Div(
                        dbc.Row(
                            [
                                dbc.Col(
                                    [
                                        dbc.Alert(
                                            [
                                                html.P("COM", style=COM_label),
                                                dbc.Input(
                                                    id="COM",
                                                    type="text",
                                                    style=COM_input,
                                                ),
                                            ],
                                            className="d-flex align-items-center",
                                            style=alerts,
                                        ),
                                        html.Div(
                                            html.P("Punkt-Datei"),
                                            style=output_label,
                                        ),
                                        dbc.Alert(
                                            [
                                                dcc.Upload(
                                                    dbc.Button(
                                                        "Browse",
                                                        color="light",
                                                        outline=True,
                                                        id="point_file_btn",
                                                        style={
                                                            "margin-right": "10px",
                                                        },
                                                    ),
                                                    id="point_file_data",
                                                    multiple=False,
                                                ),
                                                html.Div(
                                                    id="point_file_path",
                                                    style=output,
                                                ),
                                            ],
                                            className="d-flex align-items-center",
                                            style=alerts,
                                        ),
                                        html.Div(
                                            html.P("Ausgabe-Datei"),
                                            style=output_label,
                                        ),
                                        dbc.Alert(
                                            [
                                                dcc.Upload(
                                                    dbc.Button(
                                                        "Browse",
                                                        color="light",
                                                        outline=True,
                                                        id="result_file_btn",
                                                        style={
                                                            "margin-right": "10px",
                                                        },
                                                    ),
                                                    id="result_file_data",
                                                    multiple=False,
                                                ),
                                                html.Div(
                                                    id="result_file_path",
                                                    style=output,
                                                ),
                                            ],
                                            className="d-flex align-items-center",
                                            style=alerts,
                                        ),
                                    ],
                                    width=3,
                                ),
                                dbc.Col(
                                    [
                                        dbc.Button(
                                            "SWITCH FACE",
                                            color="primary",
                                            outline=False,
                                            id="switch_face_btn",
                                            style={
                                                "width": "150px",
                                                "height": "80px",
                                                "margin": "0px",
                                                "margin-bottom": "17px",
                                            },
                                        ),
                                        dbc.Button(
                                            "SEARCH TARGET",
                                            color="primary",
                                            outline=False,
                                            id="search_target_btn",
                                            style={
                                                "width": "150px",
                                                "height": "80px",
                                                "margin": "0px",
                                                "margin-bottom": "17px",
                                            },
                                        ),
                                        dbc.Button(
                                            "SET DATETIME",
                                            color="primary",
                                            outline=False,
                                            id="set_datetime_btn",
                                            style={
                                                "width": "150px",
                                                "height": "80px",
                                                "margin": "0px",
                                                "margin-bottom": "16px",
                                            },
                                        ),
                                        dbc.Button(
                                            "Don't click me!",
                                            color="danger",
                                            outline=False,
                                            id="fun_btn",
                                            style={
                                                "width": "150px",
                                                "height": "80px",
                                                "margin": "0px",
                                            },
                                        ),
                                    ],
                                    width=2,
                                ),
                                dbc.Col(
                                    [
                                        html.Div(
                                            [
                                                html.P(
                                                    "Waiting Time [sec]",
                                                    style={
                                                        "text-align": "center",
                                                        "color": "silver",
                                                        "margin-bottom": "0px",
                                                        "margin-top": "10px",
                                                    },
                                                ),
                                                dcc.Slider(
                                                    id="wait_time",
                                                    min=0,
                                                    max=300,
                                                    step=30,
                                                    value=30,
                                                    marks={
                                                        0: "0",
                                                        30: "30",
                                                        60: "60",
                                                        90: "90",
                                                        120: "120",
                                                        150: "150",
                                                        180: "180",
                                                        210: "210",
                                                        240: "240",
                                                        270: "270",
                                                        300: "300",
                                                    },
                                                ),
                                                html.P(
                                                    "Number of Epoches",
                                                    style={
                                                        "text-align": "center",
                                                        "color": "silver",
                                                        "margin-bottom": "0px",
                                                        "margin-top": "10px",
                                                    },
                                                ),
                                                dcc.Slider(
                                                    id="num_ep",
                                                    min=0,
                                                    max=10,
                                                    step=1,
                                                    value=1,
                                                ),
                                            ],
                                            style={
                                                "border": "solid 1px silver",
                                                "border-radius": "5px",
                                            },
                                        ),
                                        html.Br(),
                                        dbc.Row(
                                            [
                                                dbc.Col(
                                                    dbc.Button(
                                                        "FIND START",
                                                        color="success",
                                                        outline=False,
                                                        id="start_btn",
                                                        style={
                                                            "width": "130px",
                                                            "height": "198px",
                                                            "margin": "0px",
                                                        },
                                                    ),
                                                ),
                                                dbc.Col(
                                                    dbc.Button(
                                                        "RUN PROCESS",
                                                        color="success",
                                                        outline=False,
                                                        id="run_btn",
                                                        style={
                                                            "width": "130px",
                                                            "height": "198px",
                                                            "margin": "0px",
                                                        },
                                                    ),
                                                ),
                                            ]
                                        ),
                                    ],
                                    width=3,
                                ),
                            ],
                            justify="center",
                            style={
                                "text-align": "center",
                                "width": "100%",
                            },
                        )
                    )
                ],
            ),
        ]
    )


def callback_dash(app):
    # plan modal
    @app.callback(
        Output("plan_modal", "is_open"),
        Input("plan_btn", "n_clicks"),
    )
    def plan(_):
        button = [p["prop_id"] for p in callback_context.triggered][0]
        if "plan_btn" in button:
            return True
        else:
            return False

    # info canvas
    @app.callback(
        Output("info_canvas", "is_open"),
        Input("info_btn", "n_clicks"),
    )
    def info(_):
        button = [p["prop_id"] for p in callback_context.triggered][0]
        if "info_btn" in button:
            return True
        else:
            return False

    # search target
    @app.callback(
        Output("search_caution", "is_open"),
        Input("search_target_btn", "n_clicks"),
    )
    def search_target(_):
        click = [p["prop_id"] for p in callback_context.triggered][0]
        if "search_target_btn" in click:
            return True
        return False

    # point file path
    @app.callback(
        Output("point_file_path", "children"),
        [Input("point_file_data", "filename")],
        prevent_initial_call=True,
    )
    def update_file_path(filename):
        if filename is not None:
            return filename
        return ""

    # result file path
    @app.callback(
        Output("result_file_path", "children"),
        [Input("result_file_data", "filename")],
        prevent_initial_call=True,
    )
    def update_file_path(filename):
        if filename is not None:
            return filename
        return ""

    # laser button
    @app.callback(
        Output("laser_btn", "children"),
        Output("laser_warning", "is_open"),
        Input("laser_btn", "n_clicks"),
        Input("laser_btn", "children"),
        Input("COM", "value"),
    )
    def laser_btn(_, io, com):
        click = [p["prop_id"] for p in callback_context.triggered][0]
        if "n_clicks" in click:
            try:
                int(com)
                if io == "⚪":
                    utl.laserpointer(f"COM{com}", True)
                    return "🔴", 0
                else:
                    utl.laserpointer(f"COM{com}", False)
                    return "⚪", 0
            except:
                return "⚪", 1
        return "⚪", 0

    # fun button
    @app.callback(
        Output("fun_warning", "is_open"),
        Output("fun_spinner", "children"),
        Input("fun_btn", "n_clicks"),
        Input("COM", "value"),
    )
    def fun_btn(_, com):
        click = [p["prop_id"] for p in callback_context.triggered][0]
        if "fun_btn" in click:
            try:
                int(com)
                utl.set_ATR(f"COM{com}", 1)
                utl.fun(f"COM{com}")
                return 0, no_update
            except:
                return 1, no_update
        return 0, no_update

    # switch face button
    @app.callback(
        Output("face_warning", "is_open"),
        Output("switch_face_success", "is_open"),
        Output("switch_face_spinner", "children"),
        Input("switch_face_btn", "n_clicks"),
        Input("COM", "value"),
    )
    def switch_face_btn(_, com):
        click = [p["prop_id"] for p in callback_context.triggered][0]
        if "switch_face_btn" in click:
            try:
                int(com)
                utl.set_ATR(f"COM{com}", 1)
                utl.switch_face(f"COM{com}")
                return 0, 1, no_update
            except:
                return 1, 0, no_update
        return 0, 0, no_update

    # set datetime button
    @app.callback(
        Output("date_warning", "is_open"),
        Output("date_success", "is_open"),
        Input("set_datetime_btn", "n_clicks"),
        Input("COM", "value"),
    )
    def set_datetime_btn(_, com):
        click = [p["prop_id"] for p in callback_context.triggered][0]
        if "set_datetime_btn" in click:
            try:
                int(com)
                utl.set_datetime(f"COM{com}")
                return 0, 1
            except:
                return 1, 0
        return 0, 0

    # start button
    @app.callback(
        Output("start_warning", "is_open"),
        Output("start_success", "is_open"),
        Output("start_spinner", "children"),
        Input("start_btn", "n_clicks"),
        Input("COM", "value"),
        Input("point_file_path", "children"),
    )
    def start_btn(_, com, point_file_path):
        click = [p["prop_id"] for p in callback_context.triggered][0]
        if "start_btn" in click:
            try:
                int(com)
                utl.set_ATR(f"COM{com}", 1)
                utl.find_start(f"COM{com}", point_file_path)
                return 0, 1, no_update
            except:
                return 1, 0, no_update
        return 0, 0, no_update

    # run button
    @app.callback(
        Output("run_warning", "is_open"),
        Output("run_success", "is_open"),
        Output("run_spinner", "children"),
        Input("run_btn", "n_clicks"),
        Input("COM", "value"),
        Input("point_file_path", "children"),
        Input("result_file_path", "children"),
        Input("wait_time", "value"),
        Input("num_ep", "value"),
    )
    def run_btn(_, com, point_file_path, result_file_path, wait_time, num_ep):
        click = [p["prop_id"] for p in callback_context.triggered][0]
        if "run_btn" in click:
            try:
                int(com)
                utl.set_ATR(f"COM{com}", 1)
                utl.run(
                    f"COM{com}", point_file_path, result_file_path, wait_time, num_ep
                )
                return 0, 1, no_update
            except:
                return 1, 0, no_update
        return 0, 0, no_update


def run_dash():
    # Create app
    ex_ss = [dbc.themes.SOLAR]
    app = Dash(__name__, external_stylesheets=ex_ss)
    server = app.server

    app.title = "Tachymetersteuerung"
    app._favicon = "favicon.ico"

    # Create the layout
    app.layout = layout_dash()
    # Create the callbacks
    callback_dash(app)
    # Start app
    app.run_server(debug=False)


# Create the Tkinter window
window = tk.Tk()
window.title("Tachymetersteuerung")
window.geometry("300x150")


# Function to handle the "Open App" button click event
def open_app_clicked():
    # Start the Dash app in a separate thread
    app_thread = Thread(target=run_dash)
    app_thread.daemon = True
    app_thread.start()

    # Open the Dash app in the default web browser
    open_app()


# Create a label
label = ttk.Label(window)
label.pack(pady=20)

# Create a button to open the Dash app
open_button = ttk.Button(window, text="Open App", command=open_app_clicked)
open_button.pack()

# Run the Tkinter event loop
window.mainloop()
