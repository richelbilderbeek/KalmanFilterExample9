#ifdef _WIN32
#undef __STRICT_ANSI__
#endif

#include "qtmaindialog.h"

#include <cassert>
#include <cstdlib>

#include <boost/lexical_cast.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

#include "qwt_plot.h"
#include "qwt_plot_curve.h"
#include "qwt_plot_grid.h"
#include "qwt_plot_zoomer.h"

#include "maindialog.h"
#include "matrix.h"
#include "ui_qtmaindialog.h"

const std::vector<std::vector<double> > To2dVector(const boost::numeric::ublas::matrix<double>& m)
{
  const std::size_t n_rows = m.size1();
  const std::size_t n_cols = m.size2();
  std::vector<std::vector<double> > v(n_cols,std::vector<double>(n_rows,0.0));
  for (std::size_t row = 0; row!=n_rows; ++row)
  {
    for (std::size_t col = 0; col!=n_cols; ++col)
    {
      assert(row < m.size1());
      assert(col < m.size2());
      v[col][row] = m(row,col);
    }
  }
  return v;
}


QtMainDialog::QtMainDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::QtMainDialog),
  m_can_do_sim(false)
{
  ui->setupUi(this);

  //Connect all tables to OnAnyChange
  {
    std::vector<QTableWidget *> v = CollectMatrices();
    {
      const std::vector<QTableWidget *> w = CollectVectors();
      std::copy(w.begin(),w.end(),std::back_inserter(v));
    }
    const std::size_t sz = v.size();
    for (std::size_t i=0; i!=sz; ++i)
    {
      QObject::connect(v[i],SIGNAL(cellChanged(int,int)),this,SLOT(OnAnyChange()));
    }

  }
  QObject::connect(ui->box_n_timesteps,SIGNAL(valueChanged(int)),this,SLOT(OnAnyChange()));
  on_box_n_states_valueChanged(2); //Will change table_states[1][0] to '?'
  //Put back the legend in table_states[1][0]
  {
    QTableWidgetItem * const item = new QTableWidgetItem;
    item->setText("v");
    ui->table_states->setItem(1,0,item);
  }
  on_button_2_clicked();
}

QtMainDialog::~QtMainDialog()
{
  {
    const std::size_t sz = m_curves.size();
    for (std::size_t i=0; i!=sz; ++i) delete m_curves[i];
  }
  {
    const std::size_t sz = m_plots.size();
    for (std::size_t i=0; i!=sz; ++i) delete m_plots[i];
  }
  delete ui;
}

const std::vector<QTableWidget *> QtMainDialog::CollectMatrices() const
{
  std::vector<QTableWidget *> v;
  v.push_back(ui->table_control);
  v.push_back(ui->table_init_covariance_estimate);
  v.push_back(ui->table_measurement_noise_estimate);
  v.push_back(ui->table_observation);
  v.push_back(ui->table_process_noise_covariance_estimate);
  v.push_back(ui->table_state_transition);
  return v;
}

const std::vector<QTableWidget *> QtMainDialog::CollectVectors() const
{
  std::vector<QTableWidget *> v;
  v.push_back(ui->table_init_state_estimate);
  v.push_back(ui->table_init_state_real);
  v.push_back(ui->table_input);
  v.push_back(ui->table_real_measurement_noise);
  v.push_back(ui->table_real_process_noise);
  v.push_back(ui->table_states);
  return v;
}

const std::vector<std::string> QtMainDialog::GetLegend() const
{
  std::vector<std::string> v;
  const int sz = ui->table_states->rowCount();
  for (int i=0; i!=sz; ++i)
  {
    const QTableWidgetItem * const item = ui->table_states->item(i,0);
    const std::string s = item ? item->text().toStdString() : "?";
    v.push_back(s);
  }
  return v;
}

void QtMainDialog::MatrixToTable(const boost::numeric::ublas::matrix<double>& m, QTableWidget * const table)
{
  assert(table);
  const std::size_t n_rows = m.size1();
  const std::size_t n_cols = m.size2();
  if (table->columnCount() != static_cast<int>(m.size2()))
  {
    std::cout << "BREWAK";
  }
  assert(table->rowCount() == static_cast<int>(m.size1()));
  assert(table->columnCount() == static_cast<int>(m.size2()));
  for (std::size_t row=0; row!=n_rows; ++row)
  {
    for (std::size_t col=0; col!=n_cols; ++col)
    {
      QTableWidgetItem * const item = new QTableWidgetItem;

      item->setText(boost::lexical_cast<std::string>(m(row,col)).c_str());
      table->setItem(row,col,item);
    }
  }
  assert(boost::numeric::ublas::detail::equals(ToMatrix(table),m,0.00001,0.00001));
}


void QtMainDialog::OnAnyChange()
{
  if (!m_can_do_sim)
  {
    //Sizes may be out of synch, as OnAnyChange is called
    //every resize in on_box_n_states_valueChanged.
    //on_box_n_states_valueChanged sets m_can_do_sim to false when working,
    //and sets it to true when done
    return;
  }
  m_can_do_sim = false;
  UpdateLegends();
  m_can_do_sim = true;
  try
  {
    //Do the sim
    const boost::numeric::ublas::vector<double> init_x_real = ToVector(ui->table_init_state_real);
    const boost::numeric::ublas::vector<double> input = ToVector(ui->table_input);
    const boost::numeric::ublas::vector<double> x_real_measurement_noise = ToVector(ui->table_real_measurement_noise);
    const boost::numeric::ublas::vector<double> x_first_guess = ToVector(ui->table_init_state_estimate);
    const boost::numeric::ublas::matrix<double> p_first_guess = ToMatrix(ui->table_init_covariance_estimate);
    const boost::numeric::ublas::matrix<double> control = ToMatrix(ui->table_control);
    const boost::numeric::ublas::matrix<double> measurement_noise_estimate = ToMatrix(ui->table_measurement_noise_estimate);
    const boost::numeric::ublas::matrix<double> observation = ToMatrix(ui->table_observation);
    const boost::numeric::ublas::vector<double> real_process_noise = ToVector(ui->table_real_process_noise);
    const boost::numeric::ublas::matrix<double> process_noise_estimate = ToMatrix(ui->table_process_noise_covariance_estimate);
    const boost::numeric::ublas::matrix<double> state_transition = ToMatrix(ui->table_state_transition);
    const std::vector<std::string> state_names = GetLegend();
    const int n_timesteps_desired = ui->box_n_timesteps->value();
    assert(state_names.size() == init_x_real.size());
    const MainDialog d(
      n_timesteps_desired,
      control,
      input,
      measurement_noise_estimate,
      observation,
      p_first_guess,
      process_noise_estimate,
      state_transition,
      init_x_real,
      real_process_noise,
      state_names,
      x_first_guess,
      x_real_measurement_noise);

    //Display data
    {
      //Convert data to a collection of std::vector, for QwtPlot to read
      const boost::numeric::ublas::matrix<double>& data = d.GetData();
      //n_timesteps may differ from n_timesteps_desired, because in the actual simulation
      //the innovation variance may become degenerate
      const int n_timesteps = data.size1(); //Number of rows
      const std::vector<std::vector<double> > vs = To2dVector(data);

      //Create time series
      std::vector<double> time_series(n_timesteps,0.0);
      for (int t=0; t!=n_timesteps; ++t) time_series[t] = static_cast<double>(t);

      //Put data on curves
      assert(vs.size() == m_curves.size());
      assert(n_timesteps_desired == static_cast<int>(vs[0].size()));
      assert(n_timesteps_desired == static_cast<int>(time_series.size()));
      const std::size_t n_curves = m_curves.size();
      for (std::size_t i=0; i!=n_curves; ++i)
      {
        const std::vector<double>& v = vs[i];
        assert(n_timesteps_desired == static_cast<int>(v.size()));
        #ifdef _WIN32
        m_curves[i]->setData(new QwtPointArrayData(&time_series[0],&v[0],time_series.size()));
        #else
        m_curves[i]->setData(&time_series[0],&v[0],time_series.size());
        #endif
      }

      //Put curves in the plots
      const std::size_t n_plots = m_plots.size();
      for (std::size_t i=0; i!=n_plots; ++i)
      {
        m_plots[i]->setAxisScale(QwtPlot::xBottom,0.0,n_timesteps);
        const std::vector<double> min_values
          = {
            *std::min_element(vs[(i*3)+0].begin(),vs[(i*3)+0].end()),
            *std::min_element(vs[(i*3)+1].begin(),vs[(i*3)+1].end()),
            *std::min_element(vs[(i*3)+2].begin(),vs[(i*3)+2].end())
          };
        const double min_value = *std::min_element(min_values.begin(),min_values.end());
        const std::vector<double> max_values
          = {
            *std::max_element(vs[(i*3)+0].begin(),vs[(i*3)+0].end()),
            *std::max_element(vs[(i*3)+1].begin(),vs[(i*3)+1].end()),
            *std::max_element(vs[(i*3)+2].begin(),vs[(i*3)+2].end())
          };
        const double max_value = *std::max_element(max_values.begin(),max_values.end());
        m_plots[i]->setAxisScale(
          QwtPlot::yLeft,
          min_value == max_value ? 0.0 : min_value,
          min_value == max_value ? 1.0 : max_value
        );
        m_plots[i]->replot();
      }
    }
  }
  catch (boost::bad_lexical_cast&)
  {
    const std::vector<double> v(1,0.0);
    {
      const std::size_t sz = m_curves.size();
      for (std::size_t i=0; i!=sz; ++i)
      {
        #ifdef _WIN32
        m_curves[i]->setData(new QwtPointArrayData(&v[0],&v[0],v.size()));
        #else
        m_curves[i]->setData(&v[0],&v[0],v.size());
        #endif
      }
    }
    {
      const std::size_t sz = m_plots.size();
      for (std::size_t i=0; i!=sz; ++i)
      {
        m_plots[i]->replot();
      }
    }
  }
}

const boost::numeric::ublas::matrix<double> QtMainDialog::ToMatrix(const QTableWidget * const table)
{
  assert(table);
  const int n_rows = table->rowCount();
  const int n_cols = table->columnCount();
  boost::numeric::ublas::matrix<double> v(n_rows,n_cols);
  for(int col=0;col!=n_cols;++col)
  {
    for(int row=0;row!=n_rows;++row)
    {
      const auto item = table->item(row,col);
      if (item)
      {
        const std::string text = item->text().toStdString();
        v(row,col) = boost::lexical_cast<double>(text);
      }
      else
      {
        v(row,col) = 0.0;
      }
    }
  }
  return v;
}

const boost::numeric::ublas::vector<double> QtMainDialog::ToVector(const QTableWidget * const table)
{
  assert(table);
  assert(table->columnCount() == 1);
  const int n_rows = table->rowCount();
  boost::numeric::ublas::vector<double> v(n_rows);
  for(int row=0;row!=n_rows;++row)
  {
    const auto item = table->item(row,0);
    if (item)
    {
      const std::string text = item->text().toStdString();
      v(row) = boost::lexical_cast<double>(text);
    }
    else
    {
      v(row) = 0.0;
    }
  }
  return v;

}

void QtMainDialog::on_box_n_states_valueChanged(int arg1)
{
  const int n = arg1;
  m_can_do_sim = false;

  //Use >= so that in constructor the plots, curves, etc are created
  if (n >= ui->table_states->rowCount())
  {
    //State is added

    //Update tables
    ui->table_states->setRowCount(n);
    QTableWidgetItem * const item = new QTableWidgetItem("?");
    ui->table_states->setItem(n - 1,0,item);

    while (n > static_cast<int>(m_plots.size()))
    {
      QwtPlot * const plot = new QwtPlot;
      plot->setAxisTitle(QwtPlot::xBottom,"Time");

      QwtPlotCurve * const curve_estimate = new QwtPlotCurve;
      curve_estimate->setTitle("Estimated");
      curve_estimate->attach(plot);
      curve_estimate->setStyle(QwtPlotCurve::Lines);
      curve_estimate->setPen(QPen(QColor(255,0,0)));
      QwtPlotCurve * const curve_measured = new QwtPlotCurve;
      curve_measured->setTitle("Measured");
      curve_measured->attach(plot);
      curve_measured->setStyle(QwtPlotCurve::Lines);
      curve_measured->setPen(QPen(QColor(0,255,0)));
      QwtPlotCurve * const curve_real = new QwtPlotCurve;
      curve_real->setTitle("Real");
      curve_real->attach(plot);
      curve_real->setStyle(QwtPlotCurve::Lines);
      curve_real->setPen(QPen(QColor(0,0,255)));
      m_curves.push_back(curve_estimate);
      m_curves.push_back(curve_measured);
      m_curves.push_back(curve_real);
      m_plots.push_back(plot);

      //Add grid
      { QwtPlotGrid * const grid = new QwtPlotGrid; grid->setPen(QPen(QColor(196,196,196))); grid->attach(plot); }
      //Add zoomer
      {
        new QwtPlotZoomer(plot->canvas());
      }
      ui->layout_plots->addWidget(plot);
    }
  }
  else
  {
    while (n < static_cast<int>(m_plots.size()))
    {
      //State is removed
      ui->layout_plots->removeWidget(m_plots.back());
      delete m_plots.back();
      m_plots.pop_back();
      for (int i=0; i!=3; ++i)
      {
        //delete m_curves.back(); //Done by plot
        m_curves.pop_back();
      }
    }
  }

  //Resize Matrices
  {
    const std::vector<QTableWidget *> v = CollectMatrices();
    const std::size_t sz = v.size();
    for (std::size_t i = 0; i!=sz; ++i)
    {
      QTableWidget * const table = v[i];
      table->setColumnCount(n);
      table->setRowCount(n);
    }
  }
  //Vectors
  {
    const std::vector<QTableWidget *> v = CollectVectors();
    const std::size_t sz = v.size();
    for (std::size_t i = 0; i!=sz; ++i)
    {
      QTableWidget * const table = v[i];
      table->setRowCount(n);
    }
  }
  UpdateLegends();
  m_can_do_sim = true;
  OnAnyChange();
}

void QtMainDialog::on_button_1_clicked()
{
  ui->box_n_states->setValue(1);
  this->m_can_do_sim = false;
  //Set the variables here
  //Use examples and variables as http://greg.czerniak.info/guides/kalman1
  //Context: measuring a voltage
  ui->table_control->setItem(0,0,new QTableWidgetItem("0.0"));
  ui->table_init_covariance_estimate->setItem(0,0,new QTableWidgetItem("1.0"));
  ui->table_init_state_estimate->setItem(0,0,new QTableWidgetItem("3.0"));
  ui->table_init_state_real->setItem(0,0,new QTableWidgetItem("1.25"));
  ui->table_input->setItem(0,0,new QTableWidgetItem("0.0"));
  ui->table_measurement_noise_estimate->setItem(0,0,new QTableWidgetItem("0.1"));
  ui->table_observation->setItem(0,0,new QTableWidgetItem("1.0"));
  ui->table_process_noise_covariance_estimate->setItem(0,0,new QTableWidgetItem("0.0001"));
  ui->table_real_measurement_noise->setItem(0,0,new QTableWidgetItem("0.1"));
  ui->table_real_process_noise->setItem(0,0,new QTableWidgetItem("0.00001"));
  ui->table_states->setItem(0,0,new QTableWidgetItem("V"));
  ui->table_state_transition->setItem(0,0,new QTableWidgetItem("1.0"));
  this->m_can_do_sim = true;
  OnAnyChange();
}

void QtMainDialog::on_button_2_clicked()
{
  this->m_can_do_sim = false;
  const int n = 2;
  ui->box_n_states->setValue(n);
  //Set the variables here
  //Use example from Simon, D. Kalman Filtering. Embedded Systems Programming. June 2001
  const double acceleration = 1.0;
  const double measurement_noise = 10.0; //Called 'measnoise'
  const double accelnoise = 0.2; //Called 'accelnoise'
  const double dt = 0.1; //Timestep

  {
    //A gas pedal only influences speed
    const boost::numeric::ublas::matrix<double> m
      = Matrix::CreateMatrix(n,n, { 0.0,0.0,0.0,dt } );
    MatrixToTable(m,ui->table_control);
  }
  {
    //Just a guess
    const boost::numeric::ublas::matrix<double> m
      = Matrix::CreateMatrix(n,n, { 1.0,0.0,0.0,1.0 } );
    MatrixToTable(m,ui->table_init_covariance_estimate);
  }
  {
    //Initial state estimates are a bit off on purpose
    const boost::numeric::ublas::vector<double> v
      = Matrix::CreateVector( { 10.0,1.0 } );
    VectorToTable(v,ui->table_init_state_estimate);
  }

  {
    //From exact standstill
    const boost::numeric::ublas::vector<double> v
      = Matrix::CreateVector( { 0.0,0.0 } );
    VectorToTable(v,ui->table_init_state_real);
  }

  {
    //A gas pedal only influences speed
    const boost::numeric::ublas::vector<double> v
      = Matrix::CreateVector( { 0.0,acceleration } );
    VectorToTable(v,ui->table_input);
  }

  {
    //Only (pessimistic) normal noise in GPS, speedometer has enormous noise as if defect (yet cannot be 0.0)
    const boost::numeric::ublas::matrix<double> m
      = Matrix::CreateMatrix(n,n, { 10.0 * measurement_noise,0.0,0.0,1000000.0 } );
    MatrixToTable(m,ui->table_measurement_noise_estimate);
  }

  {
    const boost::numeric::ublas::matrix<double> m
      = Matrix::CreateMatrix(n,n, { 1.0,0.0,0.0,0.0 } ); //Only use GPS, no speedometer
    MatrixToTable(m,ui->table_observation);
  }

  {
    const boost::numeric::ublas::matrix<double> m
      = Matrix::CreateMatrix(n,n, {
        10.0 * accelnoise,
        10.0 * accelnoise,
        10.0 * accelnoise,
        10.0 * accelnoise
      } ); //Pessimistic estimate
    MatrixToTable(m,ui->table_process_noise_covariance_estimate);
  }

  {
    //Only normal noise in GPS, speedometer has enormous noise as if defect (yet cannot be 0.0)
    const boost::numeric::ublas::vector<double> v
      = Matrix::CreateVector( { measurement_noise, 1000000.0 } );
    VectorToTable(v,ui->table_real_measurement_noise);
  }

  {
    const boost::numeric::ublas::vector<double> v
      = Matrix::CreateVector( { 0.5  * accelnoise * dt * dt, accelnoise * dt} );
    VectorToTable(v,ui->table_real_process_noise);
  }

  ui->table_states->setItem(0,0,new QTableWidgetItem("x"));
  ui->table_states->setItem(1,0,new QTableWidgetItem("v"));

  {
    const boost::numeric::ublas::matrix<double> m
      = Matrix::CreateMatrix(n,n, { 1.0, 0.0, dt, 1.0 } );
    MatrixToTable(m,ui->table_state_transition);
  }

  this->m_can_do_sim = true;
  OnAnyChange();
}

void QtMainDialog::on_button_3_clicked()
{
  const int n = 4;
  ui->box_n_states->setValue(n);
  //Set the variables here
  //Use example from Simon, D. Kalman Filtering. Embedded Systems Programming. June 2001
  const double dt = 0.1; //Timestep
  const double g = 9.81; //Gravity
  const double angle = M_PI / 4.0; //Radians. 45 degrees = M_PI / 4.0 radians

  {
    //A gas pedal only influences speed
    const boost::numeric::ublas::matrix<double> m
      = Matrix::CreateMatrix(n,n,
        {
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0
        }
      );
    MatrixToTable(m,ui->table_control);
  }
  {
    //Just a guess
    const boost::numeric::ublas::matrix<double> m
      = Matrix::CreateMatrix(n,n,
        {
          1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0
        }
      );

    MatrixToTable(m,ui->table_init_covariance_estimate);
  }
  {
    //Initial state estimates are a bit off on purpose
    const boost::numeric::ublas::vector<double> v
      = Matrix::CreateVector( { 0.0, 100.0 * std::cos(angle), 500.0, 100.0 * sin(angle) } );
    VectorToTable(v,ui->table_init_state_estimate);
  }

  {
    //From exact standstill
    const boost::numeric::ublas::vector<double> v
      = Matrix::CreateVector( { 0.0, 100.0 * std::cos(angle), 0.0, 100.0 * sin(angle) } );
    VectorToTable(v,ui->table_init_state_real);
  }

  {
    //Gravity influences position and velocity in the vertical direction
    const boost::numeric::ublas::vector<double> v
      = Matrix::CreateVector( { 0.0,0.0,-0.5*g*dt*dt,-g*dt} );
    VectorToTable(v,ui->table_input);
  }

  {

    const boost::numeric::ublas::matrix<double> m
      = Matrix::CreateMatrix(n,n,
        {
          0.2, 0.0, 0.0, 0.0,
          0.0, 0.2, 0.0, 0.0,
          0.0, 0.0, 0.2, 0.0,
          0.0, 0.0, 0.0, 0.2
        }
      );
    MatrixToTable(m,ui->table_measurement_noise_estimate);
  }

  {
    const boost::numeric::ublas::matrix<double> m
      = Matrix::CreateMatrix(n,n,
        {
          1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0
        }
      );
    MatrixToTable(m,ui->table_observation);
  }

  {
    const boost::numeric::ublas::matrix<double> m
      = Matrix::CreateMatrix(n,n,
        {
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0
        }
      );
    MatrixToTable(m,ui->table_process_noise_covariance_estimate);
  }

  {
    //Only normal noise in GPS, speedometer has enormous noise as if defect (yet cannot be 0.0)
    const boost::numeric::ublas::vector<double> v
      = Matrix::CreateVector( { 30.0, 30.0, 30.0, 30.0 } );
    VectorToTable(v,ui->table_real_measurement_noise);
  }

  {
    const boost::numeric::ublas::vector<double> v
      = Matrix::CreateVector( { 0.0, 0.0, 0.0, 0.0 } );
    VectorToTable(v,ui->table_real_process_noise);
  }

  ui->table_states->setItem(0,0,new QTableWidgetItem("x"));
  ui->table_states->setItem(1,0,new QTableWidgetItem("Vx"));
  ui->table_states->setItem(2,0,new QTableWidgetItem("y"));
  ui->table_states->setItem(3,0,new QTableWidgetItem("Vy"));

  {
    const boost::numeric::ublas::matrix<double> m
      = Matrix::CreateMatrix(n,n,
        { //Beware: appears as transposition of real matrix
          1.0, 0.0, 0.0, 0.0,
           dt, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, dt , 1.0
        }
      );
    MatrixToTable(m,ui->table_state_transition);
  }

  this->m_can_do_sim = true;
  OnAnyChange();
}

void QtMainDialog::UpdateLegends()
{
  const std::vector<std::string> legend = GetLegend();
  const int n = static_cast<int>(legend.size());
  for (int i=0; i!=n; ++i)
  {
    assert(i < static_cast<int>(m_plots.size()));
    m_plots[i]->setTitle(legend[i].c_str());
    m_plots[i]->setAxisTitle(QwtPlot::yLeft,legend[i].c_str());
  }
  //Matrices
  {
    const std::vector<QTableWidget *> v = CollectMatrices();
    const std::size_t sz = v.size();
    for (std::size_t i = 0; i!=sz; ++i)
    {
      QTableWidget * const table = v[i];
      assert(table->columnCount() == static_cast<int>(legend.size()));
      assert(table->rowCount() == static_cast<int>(legend.size()));
      for (int j = 0; j!=n; ++j)
      {
        {
          QTableWidgetItem * const item = new QTableWidgetItem;
          item->setText(legend[j].c_str());
          table->setVerticalHeaderItem(j,item);
        }
        {
          QTableWidgetItem * const item = new QTableWidgetItem;
          item->setText(legend[j].c_str());
          table->setHorizontalHeaderItem(j,item);
        }
      }
      table->resizeColumnsToContents();
      table->resizeRowsToContents();
      table->setFixedSize(
        table->horizontalHeader()->length() + 2 + table->verticalHeader()->width(),
        table->verticalHeader()->length() + 2 + table->horizontalHeader()->height());
    }
  }
  //Vectors
  {
    const std::vector<QTableWidget *> v = CollectVectors();
    const std::size_t sz = v.size();
    for (std::size_t i = 0; i!=sz; ++i)
    {
      QTableWidget * const table = v[i];
      assert(table->rowCount() == static_cast<int>(legend.size()));
      for (int j = 0; j!=n; ++j)
      {
        QTableWidgetItem * const item = new QTableWidgetItem;
        item->setText(legend[j].c_str());
        table->setVerticalHeaderItem(j,item);
      }
      table->resizeColumnsToContents();
      table->resizeRowsToContents();
      table->setFixedSize(
        table->horizontalHeader()->length() + 2 + table->verticalHeader()->width(),
        table->verticalHeader()->length() + 2 + table->horizontalHeader()->height());
    }
  }
}

void QtMainDialog::VectorToTable(const boost::numeric::ublas::vector<double>& v, QTableWidget * const table)
{
  assert(table);
  if (table->rowCount() != static_cast<int>(v.size()))
  {
    std::clog << "BREKA";
  }
  assert(table->rowCount() == static_cast<int>(v.size()));
  assert(table->columnCount() == 1);
  const std::size_t sz = v.size();
  for (std::size_t i=0; i!=sz; ++i)
  {
    QTableWidgetItem * const item = new QTableWidgetItem;

    item->setText(boost::lexical_cast<std::string>(v[i]).c_str());
    table->setItem(i,0,item);
  }
  assert(boost::numeric::ublas::detail::equals(ToVector(table),v,0.00001,0.00001));

}
