/****************************************************************************
** Meta object code from reading C++ file 'PointCloudLab.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.9)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../PointCloudLab.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PointCloudLab.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.9. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_PointCloudLab_t {
    QByteArrayData data[7];
    char stringdata0[141];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PointCloudLab_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PointCloudLab_t qt_meta_stringdata_PointCloudLab = {
    {
QT_MOC_LITERAL(0, 0, 13), // "PointCloudLab"
QT_MOC_LITERAL(1, 14, 27), // "on_openFileAction_triggered"
QT_MOC_LITERAL(2, 42, 0), // ""
QT_MOC_LITERAL(3, 43, 7), // "checked"
QT_MOC_LITERAL(4, 51, 31), // "on_pushButton_pointPick_clicked"
QT_MOC_LITERAL(5, 83, 30), // "on_pushButton_areaPick_clicked"
QT_MOC_LITERAL(6, 114, 26) // "on_pushButton_drag_clicked"

    },
    "PointCloudLab\0on_openFileAction_triggered\0"
    "\0checked\0on_pushButton_pointPick_clicked\0"
    "on_pushButton_areaPick_clicked\0"
    "on_pushButton_drag_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PointCloudLab[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x0a /* Public */,
       4,    0,   37,    2, 0x0a /* Public */,
       5,    0,   38,    2, 0x0a /* Public */,
       6,    0,   39,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void PointCloudLab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PointCloudLab *_t = static_cast<PointCloudLab *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_openFileAction_triggered((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->on_pushButton_pointPick_clicked(); break;
        case 2: _t->on_pushButton_areaPick_clicked(); break;
        case 3: _t->on_pushButton_drag_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject PointCloudLab::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_PointCloudLab.data,
      qt_meta_data_PointCloudLab,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *PointCloudLab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PointCloudLab::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_PointCloudLab.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int PointCloudLab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
