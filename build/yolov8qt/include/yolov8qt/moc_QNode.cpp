/****************************************************************************
** Meta object code from reading C++ file 'QNode.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/yolov8qt/include/yolov8qt/QNode.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QNode.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QNode_t {
    QByteArrayData data[14];
    char stringdata0[121];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QNode_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QNode_t qt_meta_stringdata_QNode = {
    {
QT_MOC_LITERAL(0, 0, 5), // "QNode"
QT_MOC_LITERAL(1, 6, 11), // "rosShutdown"
QT_MOC_LITERAL(2, 18, 0), // ""
QT_MOC_LITERAL(3, 19, 12), // "DataReceived"
QT_MOC_LITERAL(4, 32, 5), // "image"
QT_MOC_LITERAL(5, 38, 3), // "fps"
QT_MOC_LITERAL(6, 42, 10), // "class_name"
QT_MOC_LITERAL(7, 53, 4), // "conf"
QT_MOC_LITERAL(8, 58, 8), // "distance"
QT_MOC_LITERAL(9, 67, 8), // "electric"
QT_MOC_LITERAL(10, 76, 5), // "state"
QT_MOC_LITERAL(11, 82, 16), // "setClickPosition"
QT_MOC_LITERAL(12, 99, 3), // "pos"
QT_MOC_LITERAL(13, 103, 17) // "continueDetection"

    },
    "QNode\0rosShutdown\0\0DataReceived\0image\0"
    "fps\0class_name\0conf\0distance\0electric\0"
    "state\0setClickPosition\0pos\0continueDetection"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QNode[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x06 /* Public */,
       3,    7,   35,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    1,   50,    2, 0x0a /* Public */,
      13,    0,   53,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QImage, QMetaType::Int, QMetaType::QString, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::QString,    4,    5,    6,    7,    8,    9,   10,

 // slots: parameters
    QMetaType::Void, QMetaType::QPoint,   12,
    QMetaType::Void,

       0        // eod
};

void QNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QNode *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->rosShutdown(); break;
        case 1: _t->DataReceived((*reinterpret_cast< const QImage(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< const QString(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4])),(*reinterpret_cast< float(*)>(_a[5])),(*reinterpret_cast< float(*)>(_a[6])),(*reinterpret_cast< const QString(*)>(_a[7]))); break;
        case 2: _t->setClickPosition((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 3: _t->continueDetection(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QNode::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QNode::rosShutdown)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (QNode::*)(const QImage & , int , const QString & , float , float , float , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QNode::DataReceived)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject QNode::staticMetaObject = { {
    &QThread::staticMetaObject,
    qt_meta_stringdata_QNode.data,
    qt_meta_data_QNode,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QNode::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QNode.stringdata0))
        return static_cast<void*>(this);
    return QThread::qt_metacast(_clname);
}

int QNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
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

// SIGNAL 0
void QNode::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void QNode::DataReceived(const QImage & _t1, int _t2, const QString & _t3, float _t4, float _t5, float _t6, const QString & _t7)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)), const_cast<void*>(reinterpret_cast<const void*>(&_t6)), const_cast<void*>(reinterpret_cast<const void*>(&_t7)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
