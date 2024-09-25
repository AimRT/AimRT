[English](CONTRIBUTING.md) | 中文

# 如何贡献

感谢您对 AimRT 的关注和支持！

我们欢迎并感激任何形式的贡献，包括但不限于提交 issue、提供改进建议、改进文档、修复错误和添加功能。
本文档旨在为您提供详细的贡献指南，以帮助您更好地参与项目。
在贡献之前，请仔细阅读本指南并确保遵循这里的规则。
我们期待与您共同努力，使这个项目变得更好！

## 在贡献代码之前

项目欢迎代码补丁，但为了确保事情得到良好协调，您应该在开始工作之前讨论任何重大变更。
建议您在 issue 跟踪器中表明您的贡献意图，可以通过[认领现有 issue](https://github.com/AimRT/AimRT/issues)或[创建新 issue](https://github.com/AimRT/AimRT/issues/new) 来实现。

### 查看 issue 跟踪器

无论您已经知道要做哪些贡献，还是正在寻找想法，[issue 跟踪器](https://github.com/AimRT/AimRT/issues)始终是您的第一个去处。
issue 会被分类以管理工作流程。

大多数 issue 都会被标记为以下工作流标签之一：

- **NeedsInvestigation**：issue 尚未完全理解，需要分析以了解根本原因。
- **NeedsDecision**：issue 相对已经理解得很好，但 AimRT 团队尚未决定解决 issue 的最佳方法。
  在编写代码之前最好等待决策。
  如果一段时间内没有决策且您有兴趣处理处于这种状态的 issue，请随时在 issue 评论中“ping”维护者。
- **NeedsFix**：issue 已完全理解，可以编写代码进行修复。

### 为任何新问题打开一个 issue

除非是非常细小的变更，否则所有贡献都应与现有 issue 有关。
请随时打开一个 issue 并讨论您的计划。
这个过程让每个人都有机会验证设计，有助于防止工作重复，确保想法符合语言和工具的目标。
在编写代码之前，还可以检查设计是否合理；代码审查工具并非用于高层次的讨论。

在提交 issue 时，请确保回答以下五个问题：

1. 您正在使用哪个版本的AimRT？
2. 您正在使用哪个操作系统和编译器？
3. 您做了什么？
4. 您期望看到什么？
5. 您实际看到的是什么？

## 贡献代码

遵循 [GitHub 流程](https://docs.github.com/en/get-started/quickstart/github-flow)来[创建 GitHub PR(Pull Request)](https://docs.github.com/en/get-started/quickstart/github-flow#create-a-pull-request)。

请记住以下几点：

- 确保您的代码符合项目的代码规范。
  这包括但不限于代码风格、注释规范等。这有助于我们维护项目的整洁性和一致性。
  最基础的，您可以在项目根目录下运行 `./format.sh` 进行格式化。
- 在提交 PR 之前，请确保您已在本地测试过您的代码。 确保代码没有明显的错误并且可以正常运行（`./test.sh`）。
- 要使用新代码更新拉取请求，只需将其推送到分支； 您可以添加更多提交，也可以 rebase 并 force-push（两种风格都可以接受）。
- 如果请求被接受，所有提交将被压缩，最终提交描述将由 PR 的标题和描述组成。
  单个提交的描述将被丢弃。 请参阅以下“编写良好的提交消息”以获取一些建议。

### 编写良好的提交消息

AimRT 中的提交消息遵循一套特定的约定，我们将在本节中讨论。

以下是一个良好的示例：
> math: improve Sin, Cos and Tan precision for very large arguments
>
> The existing implementation has poor numerical properties for large arguments, so use the McGillicutty algorithm to improve accuracy above 1e10.
>
> The algorithm is described at <https://wikipedia.org/wiki/McGillicutty_Algorithm>
>
> Fixes #159

#### 第一行

变更描述的第一行通常是一个简短的一行摘要，描述变更的内容，并以主要受影响的包为前缀。

一个经验法则是，它应该被写成完成句子 "This change modifies AimRT to _____." 这意味着它不以大写字母开头，不是一个完整的句子，而且确实概括了变更的结果。

在第一行之后空一行。

#### 主要内容

描述的其余部分应该详细说明，为变更提供上下文并解释它的作用。
像在 AimRT 中的注释一样，使用正确的标点符号写完整的句子。
不要使用 HTML、Markdown 或任何其他标记语言。
添加任何相关信息，例如如果变更影响性能，请添加基准数据。

#### 引用 issue

特殊表示法 "Fixes #12345" 将变更与 AimRT issue 跟踪器中的 issue 12345关联。
当此变更最终应用时，issue 跟踪器将自动将该 issue 标记为已修复。

## 版权声明

AimRT 代码仓库中的文件不列出作者姓名，以避免混乱并避免不断更新列表。
而您的名字将出现在变更日志中。

您贡献的新文件应使用标准版权声明：

```cpp
// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.
```

代码仓库中的文件在添加时受版权保护，年份为添加年份。
在变更文件时，请勿更新版权年份。
