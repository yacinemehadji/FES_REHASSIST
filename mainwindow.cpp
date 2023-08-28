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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "settingsdialog.h"
#include <iostream>
#include <QString>
#include <QLabel>
#include <QMessageBox>
#include <windows.h>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <NIDAQmx.h>
#include "nireader.cpp"
#include <chrono>

pthread_t   acquisitionThreadStartControl;
#define Info(message) (cout << message << endl);
float Idesiree;
uint8_t channel;

using namespace std;
\
//! [0]
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow),
    m_status(new QLabel),
    m_settings(new SettingsDialog),
    amplitude(0), frequency(0), pulsewidth(0), singlepulse(0), Timeout(0), channel(0),
//! [1]
    m_serial(new QSerialPort(this))


//! [1]
{
//! [0]
    m_ui->setupUi(this);

    m_ui->actionConnect->setEnabled(true);
    m_ui->actionDisconnect->setEnabled(false);
    m_ui->actionQuit->setEnabled(true);
    m_ui->actionConfigure->setEnabled(true);
    m_ui->statusBar->addWidget(m_status);

    initActionsConnections();

    //connect(m_serial, &QSerialPort::errorOccurred, this, &MainWindow::handleError);

//! [2]
    connect(m_serial, &QSerialPort::readyRead, this, &MainWindow::readData);

    // ask current amplitude
    connect(m_ui->CurrentAmpliButton, &QPushButton::clicked, this, &MainWindow::AskCurrentAmplitude);

    // ask pulse frequency
    connect(m_ui->PulseFrequButton, &QPushButton::clicked, this, &MainWindow::AskPulseFrequency);

    // ask pulse width
    connect(m_ui->PulseWidthButton, &QPushButton::clicked, this, &MainWindow::AskPulseWidth);

    // ask start (0<timeout<15)
    connect(m_ui->SafetyTestButton, &QPushButton::clicked, this, &MainWindow::AskSafety);

    // ask stop
    connect(m_ui->StopButton, &QPushButton::clicked, this, &MainWindow::stop);

    // ask single pulse
    connect(m_ui->SinglePulseButton, &QPushButton::clicked, this, &MainWindow::AskSinglePulse);

    // ask maximum current
    connect(m_ui->MaxCurrentButton, &QPushButton::clicked, this, &MainWindow::AskMaxCurrent);

    // ask software version
    connect(m_ui->VersionButton, &QPushButton::clicked, this, &MainWindow::AskSoftwareVersion);

    // ask status
    connect(m_ui->StatusButton, &QPushButton::clicked, this, &MainWindow::AskStatus);

    // ask wakeup
    connect(m_ui->WakeupButton, &QPushButton::clicked, this, &MainWindow::wakeup);

    // ask standby
    connect(m_ui->StandbyButton, &QPushButton::clicked, this, &MainWindow::AskStandby);

    // ask start infini
    connect(m_ui->StartButtonInfini, &QPushButton::clicked, this, &MainWindow::start);

    //ask AskStartClosedLoop
    connect(m_ui->StartClosedLoopButton, &QPushButton::clicked, this, &MainWindow::AskStartClosedLoop);

    //ask AskCurrentAmpliCL
    //connect(m_ui->CurrentAmplitudeCLButton, &QPushButton::clicked, this, &MainWindow::AskCurrentAmplitudeCL);

    //ask AskStopCL
    connect(m_ui->StopCLButton, &QPushButton::clicked, this, &MainWindow::StopFuncAcquisition);

    // handle signals from other threads
    connect(this, &MainWindow::AskStart, this, &MainWindow::start);
    connect(this, &MainWindow::AskWakeup, this, &MainWindow::wakeup);
    connect(this, &MainWindow::AskCurrentAmplitudeCL, this, &MainWindow::setCurrentAmplitudeCL);

    QGraphicsScene *scene = new QGraphicsScene(this);
    QPixmap image("C:/Users/kronig/Desktop/ems2/terminal/FES.png");
    QPixmap scaledImage = image.scaled(50, 50, Qt::KeepAspectRatio);
    QGraphicsPixmapItem *pixmapItem = new QGraphicsPixmapItem(image);
    scene->addItem(pixmapItem);
    m_ui->graphicsView->setScene(scene);

    // start acquisition
    StartAcquisition();
//! [3]
}
//! [3]

MainWindow::~MainWindow()
{
    delete m_settings;
    delete m_ui;
    StopAcquisition();

}

//! [4]
void MainWindow::openSerialPort()
{
    const SettingsDialog::Settings p = m_settings->settings();
    m_serial->setPortName(p.name);
    m_serial->setBaudRate(p.baudRate);
    m_serial->setDataBits(p.dataBits);
    m_serial->setParity(p.parity);
    m_serial->setStopBits(p.stopBits);
    m_serial->setFlowControl(p.flowControl);
    if (m_serial->open(QIODevice::ReadWrite)) {
        m_ui->actionConnect->setEnabled(false);
        m_ui->actionDisconnect->setEnabled(true);
        m_ui->actionConfigure->setEnabled(false);
        showStatusMessage(tr("connected to %1 : %2, %3, %4, %5, %6")
                          .arg(p.name).arg(p.stringBaudRate).arg(p.stringDataBits)
                          .arg(p.stringParity).arg(p.stringStopBits).arg(p.stringFlowControl));
    } else {
        QMessageBox::critical(this, tr("Error"), m_serial->errorString());

        showStatusMessage(tr("Open error"));
    }
}
//! [4]

//! [5]
void MainWindow::closeSerialPort()
{
    if (m_serial->isOpen())
        m_serial->close();
    m_ui->actionConnect->setEnabled(true);
    m_ui->actionDisconnect->setEnabled(false);
    m_ui->actionConfigure->setEnabled(true);
    showStatusMessage(tr("Disconnected"));
}
//! [5]

//! [6]

//! [7]
void MainWindow::readData()
{
    Info("Updating data");
    data = m_serial->readAll();
    dataReady = true;
}
//! [7]

//! [8]
void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), m_serial->errorString());
        closeSerialPort();
    }
}
//! [8]

void MainWindow::initActionsConnections()
{
    connect(m_ui->actionConnect, &QAction::triggered, this, &MainWindow::openSerialPort);
    connect(m_ui->actionDisconnect, &QAction::triggered, this, &MainWindow::closeSerialPort);
    connect(m_ui->actionQuit, &QAction::triggered, this, &MainWindow::close);
    connect(m_ui->actionConfigure, &QAction::triggered, m_settings, &SettingsDialog::show);
    connect(m_ui->actionAboutQt, &QAction::triggered, qApp, &QApplication::aboutQt);
}

void MainWindow::showStatusMessage(const QString &message)
{
    m_status->setText(message);
}

uint8_t MainWindow::checksum(uint8_t b1a, uint8_t b1b, uint8_t b2a, uint8_t b3a, uint8_t b3b)
{
    quint8 checksum = b1a + b1b + b2a + b3a + b3b;
    checksum &= 0x0F;
    cout << "Checksum: " << (int)checksum << endl;
    return checksum;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
QString MainWindow::getStringCurrentAmplitude(uint8_t byteAmpliude){
    QString amplitude ;
    {
        switch (byteAmpliude) {
        case 4:
            amplitude = QString("Actif (pret a fonctionner)");
            break;
        case 5:
            amplitude = QString("En Standby");
            break;
        case 6:
            amplitude = QString("Electrodes non placees");
            break;
        case 7:
            amplitude = QString("Erreur de checksum ou commande erronee");
            break;
        default:
            amplitude = QString("Status retourne inconnu");
            break;
        }
        return amplitude;
    }
}

bool MainWindow::AskCurrentAmplitude()
{
    QByteArray msgToSend;

    channel=m_ui->spinBox_channel->value();

    amplitude=m_ui->doubleSpinBox_current->value();

    uint16_t convertedCurrent = 1023 * amplitude / 150;

    uint8_t b1a = 0b1000 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b3a = ((convertedCurrent >> 3) & 0xF0) >> 4;
    uint8_t b3b = (convertedCurrent >> 3) & 0xF;
    uint8_t b2a = convertedCurrent & 0b111;
    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("AskCurrentAmplitude");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
        m_serial->waitForReadyRead(100);
    }
    if(dataReady)
    {
        dataReady = false;
        uint8_t amplitude = data[0]>> 5;
        QString stringAmplitude = getStringCurrentAmplitude(amplitude);
        Info("Status received : " << stringAmplitude.toStdString());
        m_ui->lineEdit->setText(stringAmplitude);
        data.clear();
    }

    else
    {
        Info("No data received");
    }
    return result;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
QString MainWindow::getStringPulseFrequency(uint8_t byteFrequency)
{
    QString frequency ;
    {
        switch (byteFrequency) {
        case 4:
            frequency = QString("Actif (pret a fonctionner)");
            break;
        case 5:
            frequency = QString("En Standby");
            break;
        case 6:
            frequency = QString("Electrodes non placees");
            break;
        case 7:
            frequency = QString("Erreur de checksum ou commande erronee");
            break;
        default:
            frequency = QString("Status retournee inconnu");
            break;
        }
        return frequency;
    }
}

bool MainWindow::AskPulseFrequency()
{
    QByteArray msgToSend;
    channel=m_ui->spinBox_channel->value();
    uint8_t SEL;
    frequency=m_ui->doubleSpinBox_pulseFrequency->value();

    SEL = (frequency - 10)/5;

    uint8_t b1a = 0b1010 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b3a = 0b0000;
    uint8_t b3b = SEL;            // 15, 14, 13, 12 sont codes sur 4bits pas besoin de construire le b3a.

    uint8_t b2a =  0b0000;
    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("Writing AskPulseFrequency");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
        m_serial->waitForReadyRead(100);
    }
    if(dataReady)
    {
        dataReady = false;
        uint8_t frequency = data[0]>> 5;
        QString stringFrequency = getStringPulseFrequency(frequency);
        Info("Status received : " << stringFrequency.toStdString());
        m_ui->lineEdit->setText(stringFrequency);
        data.clear();
    }

    else
    {
        Info("No data received");
    }
    return result;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
QString MainWindow::getStringPulseWidth(uint8_t byteWidth)
{
    QString width ;
    {
        switch (byteWidth) {
        case 4:
            width = QString("Actif (pret a fonctionner)");
            break;
        case 5:
            width = QString("En Standby");
            break;
        case 6:
            width = QString("Electrodes non placees");
            break;
        case 7:
            width = QString("Erreur de checksum ou commande erronee");
            break;
        default:
            width = QString("Status retourne inconnu");
            break;
        }
        return width;
    }
}

bool MainWindow::AskPulseWidth()
{
    QByteArray msgToSend;

    channel=m_ui->spinBox_channel->value();

    uint8_t SEL = 13;

    pulsewidth=m_ui->doubleSpinBox_PulseWidth->value();

    switch (pulsewidth)
    {
    case 300:
        SEL = 15;
        cout << "IN" << endl;
        break;
    case 280:
        SEL = 14;
        break;
    case 260:
        SEL = 13;
    case 240:
        SEL = 0;
        break;
    }
    uint8_t b1a = 0b1010 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b2a = 0b0001;
    uint8_t b3a = 0b0000;
    uint8_t b3b = SEL;         // 15, 14, 13, 12 sont codes sur 4bits pas besoin de construire le b3a.
    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("Writing AskPulseWidth");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
        m_serial->waitForReadyRead(100);
    }
    if(dataReady)
    {
        dataReady = false;
        uint8_t width = data[0]>> 5;
        QString stringWidth = getStringPulseWidth(width);
        Info("Status received : " << stringWidth.toStdString());
        m_ui->lineEdit->setText(stringWidth);
        data.clear();
    }
    else
    {
        Info("No data received");
    }
    return result;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
QString MainWindow::getStringSafety(uint8_t byteSafety){
    QString start ;
    {
        switch (byteSafety) {
        case 4:
            start = QString("Actif (pret a fonctionner)");
            break;
        case 5:
            start = QString("En Standby");
            break;
        case 6:
            start = QString("Electrodes non placees");
            break;
        case 7:
            start = QString("Erreur de checksum ou commande erronee");
            break;
        default:
            start = QString("Status retourne inconnu");
            break;
        }
        return start;
    }
}

bool MainWindow::AskSafety()
{
    QByteArray msgToSend;

    channel=m_ui->spinBox_channel->value();

    uint8_t SEL;

    Timeout=m_ui->spinBox_start->value();

    SEL = Timeout-1;

    uint8_t b1a = 0b1010 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b2a = 0b0010;

    // debug
    //uint8_t b3 = 15;
    //uint8_t b3a = b3 << 4;
    //uint8_t b3b = b3 & 0xF;        // 15, 14, 13, 12 sont codes sur 4bits pas besoin de construire le b3a.
    // end debug

    uint8_t b3a = (SEL & 0xF0) >> 4;
    uint8_t b3b = SEL & 0xF; //& 0b1111;        // 15, 14, 13, 12 sont codes sur 4bits pas besoin de construire le b3a.
    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("Writing AskStart");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
        m_serial->waitForReadyRead(100);
    }
    if(dataReady)
    {
        dataReady = false;
        uint8_t start = data[0]>> 5;
        QString stringStart = getStringSafety(start);
        Info("Status received : " << stringStart.toStdString());
        m_ui->lineEdit->setText(stringStart);
        data.clear();
    }
    else
    {
        Info("No data received");
    }
    return result;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
QString MainWindow::getStringStop(uint8_t byteStop){
    QString stop ;
    {
        switch (byteStop) {
        case 4:
            stop = QString("Actif (pret a fonctionner)");
            break;
        case 5:
            stop = QString("En Standby");
            break;
        case 6:
            stop = QString("Electrodes non placees");
            break;
        case 7:
            stop = QString("Erreur de checksum ou commande erronee");
            break;
        default:
            stop = QString("Status retourne inconnu");
            break;
        }
        return stop;
    }
}

bool MainWindow::stop()
{
    QByteArray msgToSend;

    channel=m_ui->spinBox_channel->value();

    uint8_t b1a = 0b1010 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b2a = 0b0011;
    uint8_t b3a = 0b0000;
    uint8_t b3b = 0b0000;

    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("Writing AskStop");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
        m_serial->waitForReadyRead(100);
    }
    if(dataReady)
    {
        dataReady = false;
        uint8_t stop = data[0]>> 5;
        QString stringStop = getStringStop(stop);
        Info("Status received : " << stringStop.toStdString());
        m_ui->lineEdit->setText(stringStop);
        data.clear();
    }
    else
    {
        Info("No data received");
    }
    return result;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
QString MainWindow::getStringSinglePulse(uint8_t bytePulse){
    QString pulse ;
    {
        switch (bytePulse) {
        case 4:
            pulse = QString("Actif (pret a fonctionner)");
            break;
        case 5:
            pulse = QString("En Standby");
            break;
        case 6:
            pulse = QString("Electrodes non placees");
            break;
        case 7:
            pulse = QString("Erreur de checksum ou commande erronee");
            break;
        default:
            pulse = QString("Status retourne inconnu");
            break;
        }
        return pulse;
    }
}

bool MainWindow::AskSinglePulse()
{
    QByteArray msgToSend;

    channel=m_ui->spinBox_channel->value();

    uint8_t b1a = 0b1010 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b2a = 0b0100;
    uint8_t b3a = 0b0000;
    uint8_t b3b = 0b0000;

    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("Writing AskSinglePulse");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
        m_serial->waitForReadyRead(100);
    }
    if(dataReady)
    {
        dataReady = false;
        uint8_t pulse = data[0]>> 5;
        QString stringPulse = getStringSinglePulse(pulse);
        Info("Status received :  " << stringPulse.toStdString());
        m_ui->lineEdit->setText(stringPulse);
        data.clear();
    }
    else
    {
        Info("No data received");
    }
    return result;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
QString MainWindow::getStringMaxCurrent(uint8_t byteCurrent){
    QString current ;
    {
        switch (byteCurrent) {
        case 0:
            current = QString("Actif (pret a fonctionner)");
            break;
        case 1:
            current = QString("En Standby");
            break;
        case 2:
            current = QString("Electrodes non placees");
            break;
        case 3:
            current = QString("Erreur de checksum ou commande erronee");
            break;
        default:
            current = QString("Status retourne inconnu");
            break;
        }
        return current;
    }
}

bool MainWindow::AskMaxCurrent()
{
    QByteArray msgToSend;

    channel=m_ui->spinBox_channel->value();

    maxCurrent=m_ui->doubleSpinBox_MaximumCurrent->value();

    uint16_t convertedCurrent = 1023 * maxCurrent / 150;

    cout << (int) convertedCurrent<< endl;

    uint8_t b1a = 0b1010 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b2a = 0b0101;
    uint8_t b3a = ((convertedCurrent >> 3) & 0xF0) >> 4;
    uint8_t b3b = (convertedCurrent >> 3) & 0xF;
    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("Writing AskCurrentMax");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
        m_serial->waitForReadyRead(100);
    }
    if(dataReady)
    {
        dataReady = false;
        uint8_t current = data[0]>> 5;
        QString stringCurrent = getStringMaxCurrent(current);
        Info("Status received :  " << stringCurrent.toStdString());
        m_ui->lineEdit->setText(stringCurrent);
        data.clear();
    }
    else
    {
        Info("No data received");
    }
    return result;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
QString MainWindow::getStringVersion(uint8_t byteVersion){
    QString version ;
    switch (byteVersion) {
    case 0:
        version = QString("Version 1.0");
        break;
    case 1:
        version = QString("Version 1.1");
        break;
    case 2:
        version = QString("Version 1.2");
        break;
    case 3:
        version = QString("Version 1.3");
        break;
    default:
        break;
    }
    return version;
}

bool MainWindow::AskSoftwareVersion()
{
    QByteArray msgToSend;
    //uint8_t donnees;
    QByteArray DataReceived;

    channel=m_ui->spinBox_channel->value();

    uint8_t b1a = 0b1010 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b2a = 0b0110;
    uint8_t b3a = 0b0000;
    uint8_t b3b = 0b0000;
    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("Writing askSoftwareVersion");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
        m_serial->waitForReadyRead(100);
    }

    if(dataReady)
    {
        dataReady = false;
        uint8_t softwareVersion = data[0]>> 5;
        QString stringVersion = getStringVersion(softwareVersion);
        Info("Software version received: " << stringVersion.toStdString());
        m_ui->lineEdit->setText(stringVersion);
        data.clear();
    }
    else
    {
        Info("No data received");
    }

    return result;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
QString MainWindow::getStringStatus(uint8_t byteStatus){
    QString status ;
    {
    switch (byteStatus) {
    case 4:
        status = QString("Actif (pret a fonctionner)");
        break;
    case 5:
        status = QString("En Standby");
        break;
    case 6:
        status = QString("Electrodes non placees");
        break;
    case 7:
        status = QString("Erreur de checksum ou commande erronee");
        break;
    default:
        status = QString("Status retourne inconnu");
        break;
        }
    return status;
    }
}
bool MainWindow::AskStatus()
{
    QByteArray msgToSend;

    channel=m_ui->spinBox_channel->value();

    uint8_t b1a = 0b1010 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b2a = 0b0111;
    uint8_t b3a = 0b0000;
    uint8_t b3b = 0b0000;

    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("Writing AskStatus");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
        m_serial->waitForReadyRead(100);
    }
    if(dataReady)
    {
        dataReady = false;
        uint8_t status = (uint8_t)data[0]>> 5;
        cout <<"le status est : " << int(status) << endl;
        QString stringStatus = getStringStatus(status);
        Info("Status received :  " << stringStatus .toStdString());
        m_ui->lineEdit->setText(stringStatus);
        data.clear();
    }
    else
    {
        Info("No data received");
    }
    return result;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
QString MainWindow::getStringWakeup(uint8_t byteWakeup){
    QString wakeup ;
    {
        switch (byteWakeup) {
        case 0:
            wakeup = QString("Actif (pret a fonctionner)");
            break;
        case 1:
            wakeup = QString("En Standby");
            break;
        case 2:
            wakeup = QString("Electrodes non placees");
            break;
        case 3:
            wakeup = QString("Erreur de checksum ou commande erronee");
            break;
        default:
            wakeup = QString("Status retourne inconnu");
            break;
        }
        return wakeup;
    }
}

bool MainWindow::wakeup()
{
    QByteArray msgToSend;

    channel=m_ui->spinBox_channel->value();

    uint8_t b1a = 0b1100 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b2a = 0b0000;
    uint8_t b3a = 0b0000;
    uint8_t b3b = 0b0000;

    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("Writing AskWakeup");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
    }

    return result;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
QString MainWindow::getStringStandby(uint8_t byteStandby){
    QString standby ;
    {
        switch (byteStandby) {
        case 0:
            standby = QString("Actif (pret a fonctionner)");
            break;
        case 1:
            standby = QString("En Standby");
            break;
        case 2:
            standby = QString("Electrodes non placees");
            break;
        case 3:
            standby = QString("Erreur de checksum ou commande erronee");
            break;
        default:
            standby = QString("Status retourne inconnu");
            break;
        }
        return standby;
    }
}

bool MainWindow::AskStandby()
{
    QByteArray msgToSend;

    channel=m_ui->spinBox_channel->value();

    uint8_t b1a = 0b1100 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b2a = 0b0001;
    uint8_t b3a = 0b0000;
    uint8_t b3b = 0b0000;

    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("Writing AskStandby");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
        m_serial->waitForReadyRead(100);
    }
    if(dataReady)
    {
        dataReady = false;
        uint8_t standby = data[0]>> 5;
        QString stringStandby = getStringStandby(standby);
        Info("Status received :  " << stringStandby.toStdString());
        m_ui->lineEdit->setText(stringStandby);
        data.clear();
    }
    else
    {
        Info("No data received");
    }
    return result;
}

QString MainWindow::getStringStart(uint8_t byteStart){
    QString start ;
    {
        switch (byteStart) {
        case 0:
            start = QString("Actif (pret a fonctionner)");
            break;
        case 1:
            start = QString("En Standby");
            break;
        case 2:
            start = QString("Electrodes non placees");
            break;
        case 3:
            start = QString("Erreur de checksum ou commande erronee");
            break;
        default:
            start = QString("Status retourne inconnu");
            break;
        }
        return start;
    }
}
bool MainWindow::start()
{
    QByteArray msgToSend;

    channel=m_ui->spinBox_channel->value();

    //cout << "IN" << endl; pour test

    uint8_t b1a = 0b1010 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b2a = 0b0010;

    // debug
    //uint8_t b3 = 15;
    //uint8_t b3a = b3 << 4;
    //uint8_t b3b = b3 & 0xF;        // 15, 14, 13, 12 sont codes sur 4bits pas besoin de construire le b3a.

    // end debug
    uint8_t b3a = 0b0000;
    uint8_t b3b = 0b1111;             // pour un timeout egale a 16 on se retrouve dan un etat infini
    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("Writing AskStart");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
        m_serial->waitForReadyRead(100);
    }

    if(dataReady)
    {
        dataReady = false;
        uint8_t start = data[0]>> 5;
        QString stringStart = getStringStart(start);
        Info("Status received :  " << stringStart.toStdString());
        m_ui->lineEdit->setText(stringStart);
        data.clear();
    }


    else
    {
        Info("No data received");
    }
    return result;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
bool MainWindow::setCurrentAmplitudeCL()
{
    float Xd;
    float Kp,Ki;
    float dt = 0.01;
    float valueSensor = (1.4 + niData[0])*10;
    m_ui->lineEdit_ClosedLoop->setText(QString::number(valueSensor));
    Kp=m_ui->doubleSpinBox_KpValue->value();
    Ki=m_ui->doubleSpinBox_KiValue->value();
    Xd=m_ui->doubleSpinBox_DesiredValue->value();
    Idesiree = Kp*(Xd - valueSensor) + Ki*(Xd - valueSensor)*dt;
    m_ui->lineEdit_CurrentEMS->setText(QString::number(Idesiree));

    QByteArray msgToSend;

    channel=m_ui->spinBox_channel->value();

    uint16_t convertedCurrent = 1023 * Idesiree / 150;
    uint8_t b1a = 0b1000 + ((channel & 0x10)>> 4);
    uint8_t b1b = channel & 0xF;
    uint8_t b3a = ((convertedCurrent >> 3) & 0xF0) >> 4;
    uint8_t b3b = (convertedCurrent >> 3) & 0xF;
    uint8_t b2a = convertedCurrent & 0b111;
    uint8_t b2b = checksum(b1a, b1b, b2a, b3a, b3b);

    // Définir les bits d'adresse du canal renvoyée pour contrôle
    //quint8 addressBits = address & 0b00011111; // AD4 AD3 AD2 AD1 AD0

    uint8_t b1   = (b1a << 4) + b1b;
    uint8_t b2  = (b2a << 4) + b2b;
    uint8_t b3   = (b3a << 4) + b3b;

    msgToSend.append(b1);
    msgToSend.append(b2);
    msgToSend.append(b3);

    Info("AskCurrentAmplitude");
    cout << (int)b1 << " " << (int)b2 << " " << (int)b3 << endl;
    bool result = m_serial->write(msgToSend);
    if(result)
    {
        Info(" Succefully sent ");
        //m_serial->waitForReadyRead(100);
    }
//    if(dataReady)
//    {
//        dataReady = false;
//        uint8_t Idesiree = data[0]>> 5;
//        QString stringAmplitude = getStringCurrentAmplitude(Idesiree);
//        Info("Status received :  " << stringAmplitude.toStdString());
//        m_ui->lineEdit->setText(stringAmplitude);
//        data.clear();
//    }

//    else
//    {
//        Info("No data received");
//    }
    return result;
}

void MainWindow::AcquiThread2Func()
{
    emit MainWindow::AskWakeup();
    emit MainWindow::AskStart();
    while (runControlThread)
    {
        emit MainWindow::AskCurrentAmplitudeCL();
        Sleep(10);
    }

}

void MainWindow::AskStartClosedLoop()
{
    runControlThread = true;
    controlThread = new std::thread(&MainWindow::AcquiThread2Func, this);
    //int result = pthread_create(&acquisitionThreadStartControl, NULL, &MainWindow::AcquiThread2Func, NULL);
}


void MainWindow::StopFuncAcquisition()
{
    runControlThread = false;
    controlThread->join();
    stop();
    // clear memory thread
    delete controlThread;
    controlThread = nullptr;
}






