/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      45,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      20,   12,   11,   11, 0x08,
      66,   12,   11,   11, 0x08,
     112,   12,   11,   11, 0x08,
     149,   12,   11,   11, 0x08,
     179,   12,   11,   11, 0x08,
     209,   12,   11,   11, 0x08,
     244,   12,   11,   11, 0x08,
     277,   11,   11,   11, 0x08,
     319,   11,   11,   11, 0x08,
     348,   11,   11,   11, 0x08,
     377,   11,   11,   11, 0x08,
     413,   11,   11,   11, 0x08,
     448,   11,   11,   11, 0x08,
     483,   11,   11,   11, 0x08,
     518,   11,   11,   11, 0x08,
     558,   12,   11,   11, 0x08,
     589,   12,   11,   11, 0x08,
     620,   12,   11,   11, 0x08,
     657,  651,   11,   11, 0x08,
     708,   12,   11,   11, 0x08,
     740,   12,   11,   11, 0x08,
     772,  651,   11,   11, 0x08,
     819,  651,   11,   11, 0x08,
     863,  651,   11,   11, 0x08,
     912,  651,   11,   11, 0x08,
     957,  651,   11,   11, 0x08,
    1002,  651,   11,   11, 0x08,
    1047,  651,   11,   11, 0x08,
    1099,  651,   11,   11, 0x08,
    1142,   11,   11,   11, 0x08,
    1178, 1172,   11,   11, 0x08,
    1213, 1209,   11,   11, 0x08,
    1235,   11,   11,   11, 0x08,
    1252,   12,   11,   11, 0x08,
    1283,   12,   11,   11, 0x08,
    1321,   12,   11,   11, 0x08,
    1357,   12,   11,   11, 0x08,
    1391,   12,   11,   11, 0x08,
    1425,   11,   11,   11, 0x08,
    1467,   12,   11,   11, 0x08,
    1501,  651,   11,   11, 0x08,
    1542,   11,   11,   11, 0x08,
    1556,   11,   11,   11, 0x08,
    1576,   11,   11,   11, 0x08,
    1598,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0checked\0"
    "on_radioButton_trigger_hardware_clicked(bool)\0"
    "on_radioButton_software_trigger_clicked(bool)\0"
    "on_radioButton_collect_clicked(bool)\0"
    "on_flashlight_l_clicked(bool)\0"
    "on_flashlight_h_clicked(bool)\0"
    "on_flashlight_manual_clicked(bool)\0"
    "on_flashlight_auto_clicked(bool)\0"
    "on_lineEdit_exposure_time_returnPressed()\0"
    "on_radioButton_raw_clicked()\0"
    "on_radioButton_bmp_clicked()\0"
    "on_pushButton_snap_catch_released()\0"
    "on_pushButton_snap_path_released()\0"
    "on_pushButton_para_load_released()\0"
    "on_pushButton_para_save_released()\0"
    "on_pushButton_para_acquiesce_released()\0"
    "on_radioButton_D_clicked(bool)\0"
    "on_radioButton_C_clicked(bool)\0"
    "on_radioButton_B_clicked(bool)\0value\0"
    "on_horizontalSlider_isp_acutance_valueChanged(int)\0"
    "on_checkBox_isp_v_clicked(bool)\0"
    "on_checkBox_isp_h_clicked(bool)\0"
    "on_contrast_horizontalSlider_valueChanged(int)\0"
    "on_gamma_horizontalSlider_valueChanged(int)\0"
    "on_horizontalSlider_saturation_valueChanged(int)\0"
    "on_horizontalSlider_gain_b_valueChanged(int)\0"
    "on_horizontalSlider_gain_g_valueChanged(int)\0"
    "on_horizontalSlider_gain_r_valueChanged(int)\0"
    "on_horizontalSlider_exposure_time_valueChanged(int)\0"
    "on_horizontalSlider_gain_valueChanged(int)\0"
    "on_AWB_once_button_released()\0index\0"
    "on_res_combobox_activated(int)\0img\0"
    "Image_process(QImage)\0camera_statues()\0"
    "on_radioButton_A_clicked(bool)\0"
    "on_exposure_mode_manual_clicked(bool)\0"
    "on_exposure_mode_auto_clicked(bool)\0"
    "on_radioButton_60HZ_clicked(bool)\0"
    "on_radioButton_50HZ_clicked(bool)\0"
    "on_software_trigger_once_button_clicked()\0"
    "on_flicker_checkBox_clicked(bool)\0"
    "on_AE_horizontalSlider_valueChanged(int)\0"
    "radioChange()\0on_SetRes_clicked()\0"
    "on_SetRes_2_clicked()\0on_LoadResButton_clicked()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_radioButton_trigger_hardware_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->on_radioButton_software_trigger_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_radioButton_collect_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_flashlight_l_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->on_flashlight_h_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->on_flashlight_manual_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->on_flashlight_auto_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_lineEdit_exposure_time_returnPressed(); break;
        case 8: _t->on_radioButton_raw_clicked(); break;
        case 9: _t->on_radioButton_bmp_clicked(); break;
        case 10: _t->on_pushButton_snap_catch_released(); break;
        case 11: _t->on_pushButton_snap_path_released(); break;
        case 12: _t->on_pushButton_para_load_released(); break;
        case 13: _t->on_pushButton_para_save_released(); break;
        case 14: _t->on_pushButton_para_acquiesce_released(); break;
        case 15: _t->on_radioButton_D_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 16: _t->on_radioButton_C_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 17: _t->on_radioButton_B_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 18: _t->on_horizontalSlider_isp_acutance_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 19: _t->on_checkBox_isp_v_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 20: _t->on_checkBox_isp_h_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 21: _t->on_contrast_horizontalSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 22: _t->on_gamma_horizontalSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 23: _t->on_horizontalSlider_saturation_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 24: _t->on_horizontalSlider_gain_b_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 25: _t->on_horizontalSlider_gain_g_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 26: _t->on_horizontalSlider_gain_r_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 27: _t->on_horizontalSlider_exposure_time_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 28: _t->on_horizontalSlider_gain_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 29: _t->on_AWB_once_button_released(); break;
        case 30: _t->on_res_combobox_activated((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 31: _t->Image_process((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 32: _t->camera_statues(); break;
        case 33: _t->on_radioButton_A_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 34: _t->on_exposure_mode_manual_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 35: _t->on_exposure_mode_auto_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 36: _t->on_radioButton_60HZ_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 37: _t->on_radioButton_50HZ_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 38: _t->on_software_trigger_once_button_clicked(); break;
        case 39: _t->on_flicker_checkBox_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 40: _t->on_AE_horizontalSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 41: _t->radioChange(); break;
        case 42: _t->on_SetRes_clicked(); break;
        case 43: _t->on_SetRes_2_clicked(); break;
        case 44: _t->on_LoadResButton_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 45)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 45;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
