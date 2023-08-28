/****************************************************************************
**
** Copyright (C) 2012 Denis Shienkov <denis.shienkov@gmail.com>
** Copyright (C) 2012 Laszlo Papp <lpapp@kde.org>
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QGraphicsScene>
#include <QString>
#include <QDebug>
#include <thread>

QT_BEGIN_NAMESPACE

class QLabel;

namespace Ui {
class MainWindow;
}

QT_END_NAMESPACE

class SettingsDialog;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    bool AskWakeup();
    bool AskStart();
    bool AskCurrentAmplitudeCL();

private slots:
    void openSerialPort();
    void closeSerialPort();
    void readData();
    void handleError(QSerialPort::SerialPortError error);


private:
    void initActionsConnections();

private:
    void showStatusMessage(const QString &message);

    Ui::MainWindow *m_ui = nullptr;
    QLabel *m_status = nullptr;
    SettingsDialog *m_settings = nullptr;
    QSerialPort *m_serial = nullptr;
    QGraphicsScene *Scene;
    QByteArray data;
    uint8_t checksum(uint8_t b1a, uint8_t b1b, uint8_t b2a, uint8_t b3a, uint8_t b3b);
    uint16_t amplitude, frequency, pulsewidth, singlepulse, maxCurrent, ClosedLoop;
    uint8_t Timeout, channel;

    bool AskCurrentAmplitude();
    QString getStringCurrentAmplitude(uint8_t byteAmpliude);

    bool AskPulseFrequency();
    QString getStringPulseFrequency(uint8_t byteFrequency);

    bool AskPulseWidth();
    QString getStringPulseWidth(uint8_t byteWidth);

    bool AskSafety();
    QString getStringSafety(uint8_t byteSafety);


    QString getStringStop(uint8_t byteStop);

    bool AskSinglePulse();
    QString getStringSinglePulse(uint8_t byteWidth);

    bool AskMaxCurrent();
    QString getStringMaxCurrent(uint8_t byteCurrent);

    bool AskSoftwareVersion();
    QString getStringVersion(uint8_t byteVersion);

    bool AskStatus();
    QString getStringStatus(uint8_t byteStatus);


    QString getStringWakeup(uint8_t byteWakeup);

    bool AskStandby();
    QString getStringStandby(uint8_t byteStandby);

    QString getStringStart(uint8_t byteStart);

    bool wakeup();
    bool start();
    bool stop();
    void AskStartClosedLoop();
    bool setCurrentAmplitudeCL();
    void AcquiThread2Func();
    void StopFuncAcquisition();
    void dataReadySignal();

    std::thread *controlThread;
    bool runControlThread = true;

    bool dataReady = false;

};

#endif // MAINWINDOW_H
