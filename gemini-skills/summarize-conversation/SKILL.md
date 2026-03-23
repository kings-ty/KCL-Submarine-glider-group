---
name: summarize-conversation
description: Summarizes the current conversation history into a markdown file. Use when the user asks to summarize, create a summary, or save the conversation, for example with commands like '/summarize', '/요약', or '대화 내용 요약해줘'.
---

# Summarize Conversation

## Overview

This skill guides the agent to create a concise summary of the current conversation and save it to a markdown file in the user's workspace.

## Workflow

Follow these steps to fulfill a user's request for a summary.

### Step 1: Acknowledge and Confirm

Acknowledge the user's request for a summary.

### Step 2: Determine Filename

Ask the user for their desired filename. Suggest a sensible default like `conversation-summary.md` if they don't have a preference.

**Example:**
- **User:** "Summarize our chat."
- **Agent:** "Of course. What would you like to name the summary file? I can use `conversation-summary.md` if you don't have a preference."

### Step 3: Generate Summary

Review the entire conversation history from the beginning. Create a structured and concise summary in Markdown format.

A good summary should include:
- The user's primary goal or problem.
- Key issues, errors, or obstacles encountered.
- Solutions proposed and decisions made.
- The final outcome or the currently planned next steps.

### Step 4: Write to File

Use the `write_file` tool to save the generated Markdown summary to the filename decided in Step 2.

### Step 5: Confirm Creation

Inform the user that the file has been successfully created, providing the filename for clarity.

**Example:**
- **Agent:** "I have saved the summary to `conversation-summary.md`."