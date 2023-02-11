//===-- ToolRunner.cpp ----------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the interfaces described in the ToolRunner.h file.
//
//===----------------------------------------------------------------------===//

#include "ToolRunner.h"
#include "llvm/Config/config.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/FileUtilities.h"
#include "llvm/Support/Program.h"
#include "llvm/Support/raw_ostream.h"
#include <fstream>
#include <sstream>
#include <utility>
using namespace llvm;

#define DEBUG_TYPE "toolrunner"

namespace llvm {
cl::opt<bool> SaveTemps("save-temps", cl::init(false),
                        cl::desc("Save temporary files"));
}

namespace {
cl::opt<std::string>
    RemoteClient("remote-client",
                 cl::desc("Remote execution client (rsh/ssh)"));

cl::opt<std::string> RemoteHost("remote-host",
                                cl::desc("Remote execution (rsh/ssh) host"));

cl::opt<std::string> RemotePort("remote-port",
                                cl::desc("Remote execution (rsh/ssh) port"));

cl::opt<std::string> RemoteUser("remote-user",
                                cl::desc("Remote execution (rsh/ssh) user id"));

cl::opt<std::string>
    RemoteExtra("remote-extra-options",
                cl::desc("Remote execution (rsh/ssh) extra options"));
}

/// RunProgramWithTimeout - This function provides an alternate interface
/// to the sys::Program::ExecuteAndWait interface.
/// @see sys::Program::ExecuteAndWait
static int RunProgramWithTimeout(StringRef ProgramPath,
                                 ArrayRef<StringRef> Args, StringRef StdInFile,
                                 StringRef StdOutFile, StringRef StdErrFile,
                                 unsigned NumSeconds = 0,
                                 unsigned MemoryLimit = 0,
                                 std::string *ErrMsg = nullptr) {
  std::optional<StringRef> Redirects[3] = {StdInFile, StdOutFile, StdErrFile};
  return sys::ExecuteAndWait(ProgramPath, Args, std::nullopt, Redirects,
                             NumSeconds, MemoryLimit, ErrMsg);
}

/// RunProgramRemotelyWithTimeout - This function runs the given program
/// remotely using the given remote client and the sys::Program::ExecuteAndWait.
/// Returns the remote program exit code or reports a remote client error if it
/// fails. Remote client is required to return 255 if it failed or program exit
/// code otherwise.
/// @see sys::Program::ExecuteAndWait
static int RunProgramRemotelyWithTimeout(
    StringRef RemoteClientPath, ArrayRef<StringRef> Args, StringRef StdInFile,
    StringRef StdOutFile, StringRef StdErrFile, unsigned NumSeconds = 0,
    unsigned MemoryLimit = 0) {
  std::optional<StringRef> Redirects[3] = {StdInFile, StdOutFile, StdErrFile};

  // Run the program remotely with the remote client
  int ReturnCode = sys::ExecuteAndWait(RemoteClientPath, Args, std::nullopt,
                                       Redirects, NumSeconds, MemoryLimit);

  // Has the remote client fail?
  if (255 == ReturnCode) {
    std::ostringstream OS;
    OS << "\nError running remote client:\n ";
    for (StringRef Arg : Args)
      OS << " " << Arg.str();
    OS << "\n";

    // The error message is in the output file, let's print it out from there.
    std::string StdOutFileName = StdOutFile.str();
    std::ifstream ErrorFile(StdOutFileName.c_str());
    if (ErrorFile) {
      std::copy(std::istreambuf_iterator<char>(ErrorFile),
                std::istreambuf_iterator<char>(),
                std::ostreambuf_iterator<char>(OS));
      ErrorFile.close();
    }

    errs() << OS.str();
  }

  return ReturnCode;
}

static Error ProcessFailure(StringRef ProgPath, ArrayRef<StringRef> Args,
                            unsigned Timeout = 0, unsigned MemoryLimit = 0) {
  std::ostringstream OS;
  OS << "\nError running tool:\n ";
  for (StringRef Arg : Args)
    OS << " " << Arg.str();
  OS << "\n";

  // Rerun the compiler, capturing any error messages to print them.
  SmallString<128> ErrorFilename;
  std::error_code EC = sys::fs::createTemporaryFile(
      "bugpoint.program_error_messages", "", ErrorFilename);
  if (EC) {
    errs() << "Error making unique filename: " << EC.message() << "\n";
    exit(1);
  }

  RunProgramWithTimeout(ProgPath, Args, "", ErrorFilename.str(),
                        ErrorFilename.str(), Timeout, MemoryLimit);
  // FIXME: check return code ?

  // Print out the error messages generated by CC if possible...
  std::ifstream ErrorFile(ErrorFilename.c_str());
  if (ErrorFile) {
    std::copy(std::istreambuf_iterator<char>(ErrorFile),
              std::istreambuf_iterator<char>(),
              std::ostreambuf_iterator<char>(OS));
    ErrorFile.close();
  }

  sys::fs::remove(ErrorFilename.c_str());
  return make_error<StringError>(OS.str(), inconvertibleErrorCode());
}

//===---------------------------------------------------------------------===//
// LLI Implementation of AbstractIntepreter interface
//
namespace {
class LLI : public AbstractInterpreter {
  std::string LLIPath;               // The path to the LLI executable
  std::vector<std::string> ToolArgs; // Args to pass to LLI
public:
  LLI(const std::string &Path, const std::vector<std::string> *Args)
      : LLIPath(Path) {
    ToolArgs.clear();
    if (Args) {
      ToolArgs = *Args;
    }
  }

  Expected<int> ExecuteProgram(
      const std::string &Bitcode, const std::vector<std::string> &Args,
      const std::string &InputFile, const std::string &OutputFile,
      const std::vector<std::string> &CCArgs,
      const std::vector<std::string> &SharedLibs = std::vector<std::string>(),
      unsigned Timeout = 0, unsigned MemoryLimit = 0) override;
};
}

Expected<int> LLI::ExecuteProgram(const std::string &Bitcode,
                                  const std::vector<std::string> &Args,
                                  const std::string &InputFile,
                                  const std::string &OutputFile,
                                  const std::vector<std::string> &CCArgs,
                                  const std::vector<std::string> &SharedLibs,
                                  unsigned Timeout, unsigned MemoryLimit) {
  std::vector<StringRef> LLIArgs;
  LLIArgs.push_back(LLIPath);
  LLIArgs.push_back("-force-interpreter=true");

  for (std::vector<std::string>::const_iterator i = SharedLibs.begin(),
                                                e = SharedLibs.end();
       i != e; ++i) {
    LLIArgs.push_back("-load");
    LLIArgs.push_back(*i);
  }

  // Add any extra LLI args.
  for (unsigned i = 0, e = ToolArgs.size(); i != e; ++i)
    LLIArgs.push_back(ToolArgs[i]);

  LLIArgs.push_back(Bitcode);
  // Add optional parameters to the running program from Argv
  for (unsigned i = 0, e = Args.size(); i != e; ++i)
    LLIArgs.push_back(Args[i]);

  outs() << "<lli>";
  outs().flush();
  LLVM_DEBUG(errs() << "\nAbout to run:\t";
             for (unsigned i = 0, e = LLIArgs.size(); i != e; ++i) errs()
             << " " << LLIArgs[i];
             errs() << "\n";);
  return RunProgramWithTimeout(LLIPath, LLIArgs, InputFile, OutputFile,
                               OutputFile, Timeout, MemoryLimit);
}

void AbstractInterpreter::anchor() {}

ErrorOr<std::string> llvm::FindProgramByName(const std::string &ExeName,
                                             const char *Argv0,
                                             void *MainAddr) {
  // Check the directory that the calling program is in.  We can do
  // this if ProgramPath contains at least one / character, indicating that it
  // is a relative path to the executable itself.
  std::string Main = sys::fs::getMainExecutable(Argv0, MainAddr);
  StringRef Result = sys::path::parent_path(Main);
  if (ErrorOr<std::string> Path = sys::findProgramByName(ExeName, Result))
    return *Path;

  // Check the user PATH.
  return sys::findProgramByName(ExeName);
}

// LLI create method - Try to find the LLI executable
AbstractInterpreter *
AbstractInterpreter::createLLI(const char *Argv0, std::string &Message,
                               const std::vector<std::string> *ToolArgs) {
  if (ErrorOr<std::string> LLIPath =
      FindProgramByName("lli", Argv0, (void *)(intptr_t)&createLLI)) {
    Message = "Found lli: " + *LLIPath + "\n";
    return new LLI(*LLIPath, ToolArgs);
  } else {
    Message = LLIPath.getError().message() + "\n";
    return nullptr;
  }
}

//===---------------------------------------------------------------------===//
// Custom compiler command implementation of AbstractIntepreter interface
//
// Allows using a custom command for compiling the bitcode, thus allows, for
// example, to compile a bitcode fragment without linking or executing, then
// using a custom wrapper script to check for compiler errors.
namespace {
class CustomCompiler : public AbstractInterpreter {
  std::string CompilerCommand;
  std::vector<std::string> CompilerArgs;

public:
  CustomCompiler(const std::string &CompilerCmd,
                 std::vector<std::string> CompArgs)
      : CompilerCommand(CompilerCmd), CompilerArgs(std::move(CompArgs)) {}

  Error compileProgram(const std::string &Bitcode, unsigned Timeout = 0,
                       unsigned MemoryLimit = 0) override;

  Expected<int> ExecuteProgram(
      const std::string &Bitcode, const std::vector<std::string> &Args,
      const std::string &InputFile, const std::string &OutputFile,
      const std::vector<std::string> &CCArgs = std::vector<std::string>(),
      const std::vector<std::string> &SharedLibs = std::vector<std::string>(),
      unsigned Timeout = 0, unsigned MemoryLimit = 0) override {
    return make_error<StringError>(
        "Execution not supported with -compile-custom",
        inconvertibleErrorCode());
  }
};
}

Error CustomCompiler::compileProgram(const std::string &Bitcode,
                                     unsigned Timeout, unsigned MemoryLimit) {

  std::vector<StringRef> ProgramArgs;
  ProgramArgs.push_back(CompilerCommand);

  for (const auto &Arg : CompilerArgs)
    ProgramArgs.push_back(Arg);
  ProgramArgs.push_back(Bitcode);

  // Add optional parameters to the running program from Argv
  for (const auto &Arg : CompilerArgs)
    ProgramArgs.push_back(Arg);

  if (RunProgramWithTimeout(CompilerCommand, ProgramArgs, "", "", "", Timeout,
                            MemoryLimit))
    return ProcessFailure(CompilerCommand, ProgramArgs, Timeout, MemoryLimit);
  return Error::success();
}

//===---------------------------------------------------------------------===//
// Custom execution command implementation of AbstractIntepreter interface
//
// Allows using a custom command for executing the bitcode, thus allows,
// for example, to invoke a cross compiler for code generation followed by
// a simulator that executes the generated binary.
namespace {
class CustomExecutor : public AbstractInterpreter {
  std::string ExecutionCommand;
  std::vector<std::string> ExecutorArgs;

public:
  CustomExecutor(const std::string &ExecutionCmd,
                 std::vector<std::string> ExecArgs)
      : ExecutionCommand(ExecutionCmd), ExecutorArgs(std::move(ExecArgs)) {}

  Expected<int> ExecuteProgram(
      const std::string &Bitcode, const std::vector<std::string> &Args,
      const std::string &InputFile, const std::string &OutputFile,
      const std::vector<std::string> &CCArgs,
      const std::vector<std::string> &SharedLibs = std::vector<std::string>(),
      unsigned Timeout = 0, unsigned MemoryLimit = 0) override;
};
}

Expected<int> CustomExecutor::ExecuteProgram(
    const std::string &Bitcode, const std::vector<std::string> &Args,
    const std::string &InputFile, const std::string &OutputFile,
    const std::vector<std::string> &CCArgs,
    const std::vector<std::string> &SharedLibs, unsigned Timeout,
    unsigned MemoryLimit) {

  std::vector<StringRef> ProgramArgs;
  ProgramArgs.push_back(ExecutionCommand);

  for (std::size_t i = 0; i < ExecutorArgs.size(); ++i)
    ProgramArgs.push_back(ExecutorArgs[i]);
  ProgramArgs.push_back(Bitcode);

  // Add optional parameters to the running program from Argv
  for (unsigned i = 0, e = Args.size(); i != e; ++i)
    ProgramArgs.push_back(Args[i]);

  return RunProgramWithTimeout(ExecutionCommand, ProgramArgs, InputFile,
                               OutputFile, OutputFile, Timeout, MemoryLimit);
}

// Tokenize the CommandLine to the command and the args to allow
// defining a full command line as the command instead of just the
// executed program. We cannot just pass the whole string after the command
// as a single argument because then the program sees only a single
// command line argument (with spaces in it: "foo bar" instead
// of "foo" and "bar").
//
// Spaces are used as a delimiter; however repeated, leading, and trailing
// whitespace are ignored. Simple escaping is allowed via the '\'
// character, as seen below:
//
// Two consecutive '\' evaluate to a single '\'.
// A space after a '\' evaluates to a space that is not interpreted as a
// delimiter.
// Any other instances of the '\' character are removed.
//
// Example:
// '\\' -> '\'
// '\ ' -> ' '
// 'exa\mple' -> 'example'
//
static void lexCommand(const char *Argv0, std::string &Message,
                       const std::string &CommandLine, std::string &CmdPath,
                       std::vector<std::string> &Args) {

  std::string Token;
  std::string Command;
  bool FoundPath = false;

  // first argument is the PATH.
  // Skip repeated whitespace, leading whitespace and trailing whitespace.
  for (std::size_t Pos = 0u; Pos <= CommandLine.size(); ++Pos) {
    if ('\\' == CommandLine[Pos]) {
      if (Pos + 1 < CommandLine.size())
        Token.push_back(CommandLine[++Pos]);

      continue;
    }
    if (' ' == CommandLine[Pos] || CommandLine.size() == Pos) {
      if (Token.empty())
        continue;

      if (!FoundPath) {
        Command = Token;
        FoundPath = true;
        Token.clear();
        continue;
      }

      Args.push_back(Token);
      Token.clear();
      continue;
    }
    Token.push_back(CommandLine[Pos]);
  }

  auto Path = FindProgramByName(Command, Argv0, (void *)(intptr_t)&lexCommand);
  if (!Path) {
    Message = std::string("Cannot find '") + Command +
              "' in PATH: " + Path.getError().message() + "\n";
    return;
  }
  CmdPath = *Path;

  Message = "Found command in: " + CmdPath + "\n";
}

// Custom execution environment create method, takes the execution command
// as arguments
AbstractInterpreter *AbstractInterpreter::createCustomCompiler(
    const char *Argv0, std::string &Message,
    const std::string &CompileCommandLine) {

  std::string CmdPath;
  std::vector<std::string> Args;
  lexCommand(Argv0, Message, CompileCommandLine, CmdPath, Args);
  if (CmdPath.empty())
    return nullptr;

  return new CustomCompiler(CmdPath, Args);
}

// Custom execution environment create method, takes the execution command
// as arguments
AbstractInterpreter *
AbstractInterpreter::createCustomExecutor(const char *Argv0,
                                          std::string &Message,
                                          const std::string &ExecCommandLine) {

  std::string CmdPath;
  std::vector<std::string> Args;
  lexCommand(Argv0, Message, ExecCommandLine, CmdPath, Args);
  if (CmdPath.empty())
    return nullptr;

  return new CustomExecutor(CmdPath, Args);
}

//===----------------------------------------------------------------------===//
// LLC Implementation of AbstractIntepreter interface
//
Expected<CC::FileType> LLC::OutputCode(const std::string &Bitcode,
                                       std::string &OutputAsmFile,
                                       unsigned Timeout, unsigned MemoryLimit) {
  const char *Suffix = (UseIntegratedAssembler ? ".llc.o" : ".llc.s");

  SmallString<128> UniqueFile;
  std::error_code EC =
      sys::fs::createUniqueFile(Bitcode + "-%%%%%%%" + Suffix, UniqueFile);
  if (EC) {
    errs() << "Error making unique filename: " << EC.message() << "\n";
    exit(1);
  }
  OutputAsmFile = std::string(UniqueFile.str());
  std::vector<StringRef> LLCArgs;
  LLCArgs.push_back(LLCPath);

  // Add any extra LLC args.
  for (unsigned i = 0, e = ToolArgs.size(); i != e; ++i)
    LLCArgs.push_back(ToolArgs[i]);

  LLCArgs.push_back("-o");
  LLCArgs.push_back(OutputAsmFile); // Output to the Asm file
  LLCArgs.push_back(Bitcode);       // This is the input bitcode

  if (UseIntegratedAssembler)
    LLCArgs.push_back("-filetype=obj");

  outs() << (UseIntegratedAssembler ? "<llc-ia>" : "<llc>");
  outs().flush();
  LLVM_DEBUG(errs() << "\nAbout to run:\t";
             for (unsigned i = 0, e = LLCArgs.size(); i != e; ++i) errs()
             << " " << LLCArgs[i];
             errs() << "\n";);
  if (RunProgramWithTimeout(LLCPath, LLCArgs, "", "", "", Timeout, MemoryLimit))
    return ProcessFailure(LLCPath, LLCArgs, Timeout, MemoryLimit);
  return UseIntegratedAssembler ? CC::ObjectFile : CC::AsmFile;
}

Error LLC::compileProgram(const std::string &Bitcode, unsigned Timeout,
                          unsigned MemoryLimit) {
  std::string OutputAsmFile;
  Expected<CC::FileType> Result =
      OutputCode(Bitcode, OutputAsmFile, Timeout, MemoryLimit);
  sys::fs::remove(OutputAsmFile);
  if (Error E = Result.takeError())
    return E;
  return Error::success();
}

Expected<int> LLC::ExecuteProgram(const std::string &Bitcode,
                                  const std::vector<std::string> &Args,
                                  const std::string &InputFile,
                                  const std::string &OutputFile,
                                  const std::vector<std::string> &ArgsForCC,
                                  const std::vector<std::string> &SharedLibs,
                                  unsigned Timeout, unsigned MemoryLimit) {

  std::string OutputAsmFile;
  Expected<CC::FileType> FileKind =
      OutputCode(Bitcode, OutputAsmFile, Timeout, MemoryLimit);
  FileRemover OutFileRemover(OutputAsmFile, !SaveTemps);
  if (Error E = FileKind.takeError())
    return std::move(E);

  std::vector<std::string> CCArgs(ArgsForCC);
  llvm::append_range(CCArgs, SharedLibs);

  // Assuming LLC worked, compile the result with CC and run it.
  return cc->ExecuteProgram(OutputAsmFile, Args, *FileKind, InputFile,
                            OutputFile, CCArgs, Timeout, MemoryLimit);
}

/// createLLC - Try to find the LLC executable
///
LLC *AbstractInterpreter::createLLC(const char *Argv0, std::string &Message,
                                    const std::string &CCBinary,
                                    const std::vector<std::string> *Args,
                                    const std::vector<std::string> *CCArgs,
                                    bool UseIntegratedAssembler) {
  ErrorOr<std::string> LLCPath =
      FindProgramByName("llc", Argv0, (void *)(intptr_t)&createLLC);
  if (!LLCPath) {
    Message = LLCPath.getError().message() + "\n";
    return nullptr;
  }

  CC *cc = CC::create(Argv0, Message, CCBinary, CCArgs);
  if (!cc) {
    errs() << Message << "\n";
    exit(1);
  }
  Message = "Found llc: " + *LLCPath + "\n";
  return new LLC(*LLCPath, cc, Args, UseIntegratedAssembler);
}

//===---------------------------------------------------------------------===//
// JIT Implementation of AbstractIntepreter interface
//
namespace {
class JIT : public AbstractInterpreter {
  std::string LLIPath;               // The path to the LLI executable
  std::vector<std::string> ToolArgs; // Args to pass to LLI
public:
  JIT(const std::string &Path, const std::vector<std::string> *Args)
      : LLIPath(Path) {
    ToolArgs.clear();
    if (Args) {
      ToolArgs = *Args;
    }
  }

  Expected<int> ExecuteProgram(
      const std::string &Bitcode, const std::vector<std::string> &Args,
      const std::string &InputFile, const std::string &OutputFile,
      const std::vector<std::string> &CCArgs = std::vector<std::string>(),
      const std::vector<std::string> &SharedLibs = std::vector<std::string>(),
      unsigned Timeout = 0, unsigned MemoryLimit = 0) override;
};
}

Expected<int> JIT::ExecuteProgram(const std::string &Bitcode,
                                  const std::vector<std::string> &Args,
                                  const std::string &InputFile,
                                  const std::string &OutputFile,
                                  const std::vector<std::string> &CCArgs,
                                  const std::vector<std::string> &SharedLibs,
                                  unsigned Timeout, unsigned MemoryLimit) {
  // Construct a vector of parameters, incorporating those from the command-line
  std::vector<StringRef> JITArgs;
  JITArgs.push_back(LLIPath);
  JITArgs.push_back("-force-interpreter=false");

  // Add any extra LLI args.
  for (unsigned i = 0, e = ToolArgs.size(); i != e; ++i)
    JITArgs.push_back(ToolArgs[i]);

  for (unsigned i = 0, e = SharedLibs.size(); i != e; ++i) {
    JITArgs.push_back("-load");
    JITArgs.push_back(SharedLibs[i]);
  }
  JITArgs.push_back(Bitcode);
  // Add optional parameters to the running program from Argv
  for (unsigned i = 0, e = Args.size(); i != e; ++i)
    JITArgs.push_back(Args[i]);

  outs() << "<jit>";
  outs().flush();
  LLVM_DEBUG(errs() << "\nAbout to run:\t";
             for (unsigned i = 0, e = JITArgs.size(); i != e; ++i) errs()
             << " " << JITArgs[i];
             errs() << "\n";);
  LLVM_DEBUG(errs() << "\nSending output to " << OutputFile << "\n");
  return RunProgramWithTimeout(LLIPath, JITArgs, InputFile, OutputFile,
                               OutputFile, Timeout, MemoryLimit);
}

/// createJIT - Try to find the LLI executable
///
AbstractInterpreter *
AbstractInterpreter::createJIT(const char *Argv0, std::string &Message,
                               const std::vector<std::string> *Args) {
  if (ErrorOr<std::string> LLIPath =
          FindProgramByName("lli", Argv0, (void *)(intptr_t)&createJIT)) {
    Message = "Found lli: " + *LLIPath + "\n";
    return new JIT(*LLIPath, Args);
  } else {
    Message = LLIPath.getError().message() + "\n";
    return nullptr;
  }
}

//===---------------------------------------------------------------------===//
// CC abstraction
//

static bool IsARMArchitecture(std::vector<StringRef> Args) {
  for (size_t I = 0; I < Args.size(); ++I) {
    if (!Args[I].equals_insensitive("-arch"))
      continue;
    ++I;
    if (I == Args.size())
      break;
    if (Args[I].startswith_insensitive("arm"))
      return true;
  }

  return false;
}

Expected<int> CC::ExecuteProgram(const std::string &ProgramFile,
                                 const std::vector<std::string> &Args,
                                 FileType fileType,
                                 const std::string &InputFile,
                                 const std::string &OutputFile,
                                 const std::vector<std::string> &ArgsForCC,
                                 unsigned Timeout, unsigned MemoryLimit) {
  std::vector<StringRef> CCArgs;

  CCArgs.push_back(CCPath);

  if (TargetTriple.getArch() == Triple::x86)
    CCArgs.push_back("-m32");

  for (std::vector<std::string>::const_iterator I = ccArgs.begin(),
                                                E = ccArgs.end();
       I != E; ++I)
    CCArgs.push_back(*I);

  // Specify -x explicitly in case the extension is wonky
  if (fileType != ObjectFile) {
    CCArgs.push_back("-x");
    if (fileType == CFile) {
      CCArgs.push_back("c");
      CCArgs.push_back("-fno-strict-aliasing");
    } else {
      CCArgs.push_back("assembler");

      // For ARM architectures we don't want this flag. bugpoint isn't
      // explicitly told what architecture it is working on, so we get
      // it from cc flags
      if (TargetTriple.isOSDarwin() && !IsARMArchitecture(CCArgs))
        CCArgs.push_back("-force_cpusubtype_ALL");
    }
  }

  CCArgs.push_back(ProgramFile); // Specify the input filename.

  CCArgs.push_back("-x");
  CCArgs.push_back("none");
  CCArgs.push_back("-o");

  SmallString<128> OutputBinary;
  std::error_code EC =
      sys::fs::createUniqueFile(ProgramFile + "-%%%%%%%.cc.exe", OutputBinary);
  if (EC) {
    errs() << "Error making unique filename: " << EC.message() << "\n";
    exit(1);
  }
  CCArgs.push_back(OutputBinary); // Output to the right file...

  // Add any arguments intended for CC. We locate them here because this is
  // most likely -L and -l options that need to come before other libraries but
  // after the source. Other options won't be sensitive to placement on the
  // command line, so this should be safe.
  for (unsigned i = 0, e = ArgsForCC.size(); i != e; ++i)
    CCArgs.push_back(ArgsForCC[i]);

  CCArgs.push_back("-lm"); // Hard-code the math library...
  CCArgs.push_back("-O2"); // Optimize the program a bit...
  if (TargetTriple.getArch() == Triple::sparc)
    CCArgs.push_back("-mcpu=v9");

  outs() << "<CC>";
  outs().flush();
  LLVM_DEBUG(errs() << "\nAbout to run:\t";
             for (unsigned i = 0, e = CCArgs.size(); i != e; ++i) errs()
             << " " << CCArgs[i];
             errs() << "\n";);
  if (RunProgramWithTimeout(CCPath, CCArgs, "", "", ""))
    return ProcessFailure(CCPath, CCArgs);

  std::vector<StringRef> ProgramArgs;

  // Declared here so that the destructor only runs after
  // ProgramArgs is used.
  std::string Exec;

  if (RemoteClientPath.empty())
    ProgramArgs.push_back(OutputBinary);
  else {
    ProgramArgs.push_back(RemoteClientPath);
    ProgramArgs.push_back(RemoteHost);
    if (!RemoteUser.empty()) {
      ProgramArgs.push_back("-l");
      ProgramArgs.push_back(RemoteUser);
    }
    if (!RemotePort.empty()) {
      ProgramArgs.push_back("-p");
      ProgramArgs.push_back(RemotePort);
    }
    if (!RemoteExtra.empty()) {
      ProgramArgs.push_back(RemoteExtra);
    }

    // Full path to the binary. We need to cd to the exec directory because
    // there is a dylib there that the exec expects to find in the CWD
    char *env_pwd = getenv("PWD");
    Exec = "cd ";
    Exec += env_pwd;
    Exec += "; ./";
    Exec += OutputBinary.c_str();
    ProgramArgs.push_back(Exec);
  }

  // Add optional parameters to the running program from Argv
  for (unsigned i = 0, e = Args.size(); i != e; ++i)
    ProgramArgs.push_back(Args[i]);

  // Now that we have a binary, run it!
  outs() << "<program>";
  outs().flush();
  LLVM_DEBUG(
      errs() << "\nAbout to run:\t";
      for (unsigned i = 0, e = ProgramArgs.size(); i != e; ++i) errs()
      << " " << ProgramArgs[i];
      errs() << "\n";);

  FileRemover OutputBinaryRemover(OutputBinary.str(), !SaveTemps);

  if (RemoteClientPath.empty()) {
    LLVM_DEBUG(errs() << "<run locally>");
    std::string Error;
    int ExitCode = RunProgramWithTimeout(OutputBinary.str(), ProgramArgs,
                                         InputFile, OutputFile, OutputFile,
                                         Timeout, MemoryLimit, &Error);
    // Treat a signal (usually SIGSEGV) or timeout as part of the program output
    // so that crash-causing miscompilation is handled seamlessly.
    if (ExitCode < -1) {
      std::ofstream outFile(OutputFile.c_str(), std::ios_base::app);
      outFile << Error << '\n';
      outFile.close();
    }
    return ExitCode;
  } else {
    outs() << "<run remotely>";
    outs().flush();
    return RunProgramRemotelyWithTimeout(RemoteClientPath, ProgramArgs,
                                         InputFile, OutputFile, OutputFile,
                                         Timeout, MemoryLimit);
  }
}

Error CC::MakeSharedObject(const std::string &InputFile, FileType fileType,
                           std::string &OutputFile,
                           const std::vector<std::string> &ArgsForCC) {
  SmallString<128> UniqueFilename;
  std::error_code EC = sys::fs::createUniqueFile(
      InputFile + "-%%%%%%%" + LTDL_SHLIB_EXT, UniqueFilename);
  if (EC) {
    errs() << "Error making unique filename: " << EC.message() << "\n";
    exit(1);
  }
  OutputFile = std::string(UniqueFilename.str());

  std::vector<StringRef> CCArgs;

  CCArgs.push_back(CCPath);

  if (TargetTriple.getArch() == Triple::x86)
    CCArgs.push_back("-m32");

  for (std::vector<std::string>::const_iterator I = ccArgs.begin(),
                                                E = ccArgs.end();
       I != E; ++I)
    CCArgs.push_back(*I);

  // Compile the C/asm file into a shared object
  if (fileType != ObjectFile) {
    CCArgs.push_back("-x");
    CCArgs.push_back(fileType == AsmFile ? "assembler" : "c");
  }
  CCArgs.push_back("-fno-strict-aliasing");
  CCArgs.push_back(InputFile); // Specify the input filename.
  CCArgs.push_back("-x");
  CCArgs.push_back("none");
  if (TargetTriple.getArch() == Triple::sparc)
    CCArgs.push_back("-G"); // Compile a shared library, `-G' for Sparc
  else if (TargetTriple.isOSDarwin()) {
    // link all source files into a single module in data segment, rather than
    // generating blocks. dynamic_lookup requires that you set
    // MACOSX_DEPLOYMENT_TARGET=10.3 in your env.  FIXME: it would be better for
    // bugpoint to just pass that in the environment of CC.
    CCArgs.push_back("-single_module");
    CCArgs.push_back("-dynamiclib"); // `-dynamiclib' for MacOS X/PowerPC
    CCArgs.push_back("-undefined");
    CCArgs.push_back("dynamic_lookup");
  } else
    CCArgs.push_back("-shared"); // `-shared' for Linux/X86, maybe others

  if (TargetTriple.getArch() == Triple::x86_64)
    CCArgs.push_back("-fPIC"); // Requires shared objs to contain PIC

  if (TargetTriple.getArch() == Triple::sparc)
    CCArgs.push_back("-mcpu=v9");

  CCArgs.push_back("-o");
  CCArgs.push_back(OutputFile);         // Output to the right filename.
  CCArgs.push_back("-O2");              // Optimize the program a bit.

  // Add any arguments intended for CC. We locate them here because this is
  // most likely -L and -l options that need to come before other libraries but
  // after the source. Other options won't be sensitive to placement on the
  // command line, so this should be safe.
  for (unsigned i = 0, e = ArgsForCC.size(); i != e; ++i)
    CCArgs.push_back(ArgsForCC[i]);

  outs() << "<CC>";
  outs().flush();
  LLVM_DEBUG(errs() << "\nAbout to run:\t";
             for (unsigned i = 0, e = CCArgs.size(); i != e; ++i) errs()
             << " " << CCArgs[i];
             errs() << "\n";);
  if (RunProgramWithTimeout(CCPath, CCArgs, "", "", ""))
    return ProcessFailure(CCPath, CCArgs);
  return Error::success();
}

/// create - Try to find the CC executable
///
CC *CC::create(const char *Argv0, std::string &Message,
               const std::string &CCBinary,
               const std::vector<std::string> *Args) {
  auto CCPath = FindProgramByName(CCBinary, Argv0, (void *)(intptr_t)&create);
  if (!CCPath) {
    Message = "Cannot find `" + CCBinary + "' in PATH: " +
              CCPath.getError().message() + "\n";
    return nullptr;
  }

  std::string RemoteClientPath;
  if (!RemoteClient.empty()) {
    auto Path = sys::findProgramByName(RemoteClient);
    if (!Path) {
      Message = "Cannot find `" + RemoteClient + "' in PATH: " +
                Path.getError().message() + "\n";
      return nullptr;
    }
    RemoteClientPath = *Path;
  }

  Message = "Found CC: " + *CCPath + "\n";
  return new CC(*CCPath, RemoteClientPath, Args);
}
