QT += widgets serialport
requires(qtConfig(combobox))

TARGET = terminal
TEMPLATE = app

QMAKE_LFLAGS += -static

LIBS += -L$${_PRO_FILE_PWD_}/lib -lNIDAQmx

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    nireader.cpp \
    settingsdialog.cpp

HEADERS += \
    NIDAQmx.h \
    mainwindow.h \
    settingsdialog.h

FORMS += \
    mainwindow.ui \
    settingsdialog.ui

RESOURCES += \
    terminal.qrc

target.path = $$[QT_INSTALL_EXAMPLES]/serialport/terminal

INSTALLS += target

