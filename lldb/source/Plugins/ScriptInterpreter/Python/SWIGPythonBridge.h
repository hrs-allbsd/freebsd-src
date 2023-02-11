//===-- ScriptInterpreterPython.h -------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLDB_PLUGINS_SCRIPTINTERPRETER_PYTHON_SWIGPYTHONBRIDGE_H
#define LLDB_PLUGINS_SCRIPTINTERPRETER_PYTHON_SWIGPYTHONBRIDGE_H

#include <optional>
#include <string>

#include "lldb/Host/Config.h"

#if LLDB_ENABLE_PYTHON

// LLDB Python header must be included first
#include "lldb-python.h"

#include "Plugins/ScriptInterpreter/Python/PythonDataObjects.h"
#include "lldb/lldb-forward.h"
#include "lldb/lldb-types.h"
#include "llvm/Support/Error.h"

namespace lldb {
class SBEvent;
class SBCommandReturnObject;
class SBValue;
class SBStream;
class SBStructuredData;
} // namespace lldb

namespace lldb_private {
namespace python {

typedef struct swig_type_info swig_type_info;

python::PythonObject ToSWIGHelper(void *obj, swig_type_info *info);

/// A class that automatically clears an SB object when it goes out of scope.
/// Use for cases where the SB object points to a temporary/unowned entity.
template <typename T> class ScopedPythonObject : PythonObject {
public:
  ScopedPythonObject(T *sb, swig_type_info *info)
      : PythonObject(ToSWIGHelper(sb, info)), m_sb(sb) {}
  ~ScopedPythonObject() {
    if (m_sb)
      *m_sb = T();
  }
  ScopedPythonObject(ScopedPythonObject &&rhs)
      : PythonObject(std::move(rhs)), m_sb(std::exchange(rhs.m_sb, nullptr)) {}
  ScopedPythonObject(const ScopedPythonObject &) = delete;
  ScopedPythonObject &operator=(const ScopedPythonObject &) = delete;
  ScopedPythonObject &operator=(ScopedPythonObject &&) = delete;

  const PythonObject &obj() const { return *this; }

private:
  T *m_sb;
};

PythonObject ToSWIGWrapper(lldb::ValueObjectSP value_sp);
PythonObject ToSWIGWrapper(lldb::TargetSP target_sp);
PythonObject ToSWIGWrapper(lldb::ProcessSP process_sp);
PythonObject ToSWIGWrapper(lldb::ThreadPlanSP thread_plan_sp);
PythonObject ToSWIGWrapper(lldb::BreakpointSP breakpoint_sp);
PythonObject ToSWIGWrapper(const Status &status);
PythonObject ToSWIGWrapper(const StructuredDataImpl &data_impl);
PythonObject ToSWIGWrapper(lldb::ThreadSP thread_sp);
PythonObject ToSWIGWrapper(lldb::StackFrameSP frame_sp);
PythonObject ToSWIGWrapper(lldb::DebuggerSP debugger_sp);
PythonObject ToSWIGWrapper(lldb::WatchpointSP watchpoint_sp);
PythonObject ToSWIGWrapper(lldb::BreakpointLocationSP bp_loc_sp);
PythonObject ToSWIGWrapper(lldb::ExecutionContextRefSP ctx_sp);
PythonObject ToSWIGWrapper(const TypeSummaryOptions &summary_options);
PythonObject ToSWIGWrapper(const SymbolContext &sym_ctx);

PythonObject ToSWIGWrapper(std::unique_ptr<lldb::SBValue> value_sb);
PythonObject ToSWIGWrapper(std::unique_ptr<lldb::SBStream> stream_sb);
PythonObject ToSWIGWrapper(std::unique_ptr<lldb::SBStructuredData> data_sb);

python::ScopedPythonObject<lldb::SBCommandReturnObject>
ToSWIGWrapper(CommandReturnObject &cmd_retobj);
python::ScopedPythonObject<lldb::SBEvent> ToSWIGWrapper(Event *event);

} // namespace python

void *LLDBSWIGPython_CastPyObjectToSBData(PyObject *data);
void *LLDBSWIGPython_CastPyObjectToSBError(PyObject *data);
void *LLDBSWIGPython_CastPyObjectToSBValue(PyObject *data);
void *LLDBSWIGPython_CastPyObjectToSBMemoryRegionInfo(PyObject *data);

// These prototypes are the Pythonic implementations of the required callbacks.
// Although these are scripting-language specific, their definition depends on
// the public API.

python::PythonObject LLDBSwigPythonCreateScriptedObject(
    const char *python_class_name, const char *session_dictionary_name,
    lldb::ExecutionContextRefSP exe_ctx_sp,
    const lldb_private::StructuredDataImpl &args_impl,
    std::string &error_string);

llvm::Expected<bool> LLDBSwigPythonBreakpointCallbackFunction(
    const char *python_function_name, const char *session_dictionary_name,
    const lldb::StackFrameSP &sb_frame,
    const lldb::BreakpointLocationSP &sb_bp_loc,
    const lldb_private::StructuredDataImpl &args_impl);

bool LLDBSwigPythonWatchpointCallbackFunction(
    const char *python_function_name, const char *session_dictionary_name,
    const lldb::StackFrameSP &sb_frame, const lldb::WatchpointSP &sb_wp);

bool LLDBSwigPythonFormatterCallbackFunction(
    const char *python_function_name, const char *session_dictionary_name,
    lldb::TypeImplSP type_impl_sp);

bool LLDBSwigPythonCallTypeScript(const char *python_function_name,
                                  const void *session_dictionary,
                                  const lldb::ValueObjectSP &valobj_sp,
                                  void **pyfunct_wrapper,
                                  const lldb::TypeSummaryOptionsSP &options_sp,
                                  std::string &retval);

python::PythonObject
LLDBSwigPythonCreateSyntheticProvider(const char *python_class_name,
                                      const char *session_dictionary_name,
                                      const lldb::ValueObjectSP &valobj_sp);

python::PythonObject
LLDBSwigPythonCreateCommandObject(const char *python_class_name,
                                  const char *session_dictionary_name,
                                  lldb::DebuggerSP debugger_sp);

python::PythonObject LLDBSwigPythonCreateScriptedThreadPlan(
    const char *python_class_name, const char *session_dictionary_name,
    const StructuredDataImpl &args_data, std::string &error_string,
    const lldb::ThreadPlanSP &thread_plan_sp);

bool LLDBSWIGPythonCallThreadPlan(void *implementor, const char *method_name,
                                  lldb_private::Event *event_sp,
                                  bool &got_error);

python::PythonObject LLDBSwigPythonCreateScriptedBreakpointResolver(
    const char *python_class_name, const char *session_dictionary_name,
    const StructuredDataImpl &args, const lldb::BreakpointSP &bkpt_sp);

unsigned int
LLDBSwigPythonCallBreakpointResolver(void *implementor, const char *method_name,
                                     lldb_private::SymbolContext *sym_ctx);

python::PythonObject LLDBSwigPythonCreateScriptedStopHook(
    lldb::TargetSP target_sp, const char *python_class_name,
    const char *session_dictionary_name, const StructuredDataImpl &args,
    lldb_private::Status &error);

bool LLDBSwigPythonStopHookCallHandleStop(void *implementor,
                                          lldb::ExecutionContextRefSP exc_ctx,
                                          lldb::StreamSP stream);

size_t LLDBSwigPython_CalculateNumChildren(PyObject *implementor, uint32_t max);

PyObject *LLDBSwigPython_GetChildAtIndex(PyObject *implementor, uint32_t idx);

int LLDBSwigPython_GetIndexOfChildWithName(PyObject *implementor,
                                           const char *child_name);

lldb::ValueObjectSP LLDBSWIGPython_GetValueObjectSPFromSBValue(void *data);

bool LLDBSwigPython_UpdateSynthProviderInstance(PyObject *implementor);

bool LLDBSwigPython_MightHaveChildrenSynthProviderInstance(
    PyObject *implementor);

PyObject *LLDBSwigPython_GetValueSynthProviderInstance(PyObject *implementor);

bool LLDBSwigPythonCallCommand(const char *python_function_name,
                               const char *session_dictionary_name,
                               lldb::DebuggerSP debugger, const char *args,
                               lldb_private::CommandReturnObject &cmd_retobj,
                               lldb::ExecutionContextRefSP exe_ctx_ref_sp);

bool LLDBSwigPythonCallCommandObject(
    PyObject *implementor, lldb::DebuggerSP debugger, const char *args,
    lldb_private::CommandReturnObject &cmd_retobj,
    lldb::ExecutionContextRefSP exe_ctx_ref_sp);

bool LLDBSwigPythonCallModuleInit(const char *python_module_name,
                                  const char *session_dictionary_name,
                                  lldb::DebuggerSP debugger);

python::PythonObject
LLDBSWIGPythonCreateOSPlugin(const char *python_class_name,
                             const char *session_dictionary_name,
                             const lldb::ProcessSP &process_sp);

python::PythonObject
LLDBSWIGPython_CreateFrameRecognizer(const char *python_class_name,
                                     const char *session_dictionary_name);

PyObject *
LLDBSwigPython_GetRecognizedArguments(PyObject *implementor,
                                      const lldb::StackFrameSP &frame_sp);

bool LLDBSWIGPythonRunScriptKeywordProcess(const char *python_function_name,
                                           const char *session_dictionary_name,
                                           const lldb::ProcessSP &process,
                                           std::string &output);

std::optional<std::string>
LLDBSWIGPythonRunScriptKeywordThread(const char *python_function_name,
                                     const char *session_dictionary_name,
                                     lldb::ThreadSP thread);

bool LLDBSWIGPythonRunScriptKeywordTarget(const char *python_function_name,
                                          const char *session_dictionary_name,
                                          const lldb::TargetSP &target,
                                          std::string &output);

std::optional<std::string>
LLDBSWIGPythonRunScriptKeywordFrame(const char *python_function_name,
                                    const char *session_dictionary_name,
                                    lldb::StackFrameSP frame);

bool LLDBSWIGPythonRunScriptKeywordValue(const char *python_function_name,
                                         const char *session_dictionary_name,
                                         const lldb::ValueObjectSP &value,
                                         std::string &output);

void *LLDBSWIGPython_GetDynamicSetting(void *module, const char *setting,
                                       const lldb::TargetSP &target_sp);

} // namespace lldb_private

#endif // LLDB_ENABLE_PYTHON
#endif // LLDB_PLUGINS_SCRIPTINTERPRETER_PYTHON_SWIGPYTHONBRIDGE_H
