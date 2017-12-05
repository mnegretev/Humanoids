#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QtWidgets/QFileDialog>

#define TOTAL_REGISTERS 40

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    connected = false;
    ui->setupUi(this);
    QStringList tableHorizontalLabels;
    tableHorizontalLabels << "ADDR" << "DESCRIPTION" << "VAL";
    ui->tableMain->setRowCount(TOTAL_REGISTERS);
    ui->tableMain->setColumnCount(3);
    ui->tableMain->setHorizontalHeaderLabels(tableHorizontalLabels);
    ui->tableMain->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->tableMain->setSelectionMode(QAbstractItemView::SingleSelection);
    
    for(int i=0; i < TOTAL_REGISTERS; i++)
	ui->tableMain->setItem(i, 0, new QTableWidgetItem(QString::number(i)));

    ui->tableMain->setItem(0,1, new QTableWidgetItem("Model Number (L)"));
    ui->tableMain->setItem(1,1, new QTableWidgetItem("Model Number (H)"));
    ui->tableMain->setItem(2,1, new QTableWidgetItem("Firmware version"));
    ui->tableMain->setItem(3,1, new QTableWidgetItem("Motor ID"));
    ui->tableMain->setItem(4,1, new QTableWidgetItem("Baud Rate"));

    ui->tableMain->setItem(0,2, new QTableWidgetItem("255"));

    ui->tableMain->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);
    ui->tableMain->horizontalHeader()->resizeSection(1,ui->tableMain->width() -
						     ui->tableMain->horizontalHeader()->sectionSize(0) -
						     ui->tableMain->horizontalHeader()->sectionSize(2) - 18);
    
    for(int i=0; i < TOTAL_REGISTERS; i++)
	for(int j=0; j < 3; j++)
	{
	    QTableWidgetItem* it;
	    it = ui->tableMain->item(i,j);
	    if(it != NULL)
		it->setTextAlignment(Qt::AlignCenter);
	    
	}
    ui->listMain->addItem("No servos found");
    
    QObject::connect(ui->tableMain, SIGNAL(cellClicked(int, int)), this, SLOT(tableCellSelected(int, int)));
    QObject::connect(ui->btnConnect, SIGNAL(clicked()), this, SLOT(btnConnectClicked()));
    QObject::connect(ui->btnSearch, SIGNAL(clicked()), this, SLOT(btnSearchClicked()));

    ui->tableMain->setEnabled(false);
    ui->listMain->setEnabled(false);
    ui->btnSearch->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::tableCellSelected(int row, int col)
{
    std::cout << "Selected cell: " << row << "\t" << col << std::endl;
}

void MainWindow::btnConnectClicked()
{
    if(!connected)
    {
	connected = true;
	ui->tableMain->setEnabled(true);
	ui->listMain->setEnabled(true);
	ui->btnSearch->setEnabled(true);
    } else
    {
	connected = false;
	ui->tableMain->setEnabled(false);
	ui->listMain->setEnabled(false);
	ui->btnSearch->setEnabled(false);
    }
    
}

void MainWindow::btnSearchClicked()
{
    for(int i=0; i < 5; i++)
	ui->listMain->addItem("MX-106T");
}
