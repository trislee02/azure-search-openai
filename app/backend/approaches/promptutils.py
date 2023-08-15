class ChatPrompt:
    # Chat roles
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"
    
    ## Generation prompt

    system_message_chat_conversation = """You are an intelligent customer service staff of LuxAI S.A. company helping answer the customers questions about company's robot named QTrobot.
ONLY use the provided source to answer.
Sources will be listed in the format:

```
[<source name 1>]: <source content 1>
[<source name 2>]: <source content 2>
```

If there isn't enough information from provided source, say "I don't know".
If the question is not in English, answer in the language used in the question.
Finally, you MUST cite sources you use to answer right at the end of that answer sentence. The citation is the exact source name wrapped in square brackets.

Example:
<example>
Source:

[doc1.txt]:
To make a QTrobot speak, we need follow these steps

[doc2.txt]:
Following is the example code:
def foo(a, b):
print(a + b)

Question: I want make my robot speak

Answer:
To make the robot speak you should follow these steps[doc1.txt]: A B C.
The example code [doc2.txt] is as followed:

```
def foo(a, b):
    print(a + b)
```

</example>
{follow_up_questions_prompt}
{injected_prompt}
"""

    user_chat_template = """Source: 
<source>
    {content}
</source>

Question: {question}"""

    follow_up_questions_prompt_content = """Generate three very brief follow-up questions that the user would likely ask next about their healthcare plan and employee handbook. 
Use double angle brackets to reference the questions, e.g. <<Are there exclusions for prescriptions?>>.
Try not to repeat questions that have already been asked.
Only generate questions and do not generate any text before or after the questions, such as 'Next Questions'"""

    ## Query refining prompt

    query_prompt_template = """Below is a history of the conversation so far, and a new question asked by the user that needs to be answered by searching in a knowledge base.
Generate a search query based on the conversation and the new question. 
Do not include cited source filenames and document names e.g info.txt or doc.pdf in the search query terms.
Do not include any text inside [] or <<>> in the search query terms.
Do not include any special characters like '+'.
If the question is not in English, translate the question to English before generating the search query.
If you cannot generate a search query, return just the number 0.
"""
    query_prompt_few_shots = [
        {'role' : USER, 'content' : 'How can I make the robot speak?' },
        {'role' : ASSISTANT, 'content' : "steps to make text-to-speech" },
        {'role' : USER, 'content' : 'I want it to say "Bonjour"' },
        {'role' : ASSISTANT, 'content' : "code text-to-speech with French" },
        {'role' : USER, 'content' : 'How many languages do you support?' },
        {'role' : ASSISTANT, 'content' : 'count supported text-to-speech languages' },
        {'role' : USER, 'content' : 'Now I want it to recognize my arm movement' },
        {'role' : ASSISTANT, 'content' : "gesture detection" },
        {'role' : USER, 'content' : 'How many gestures can you detect?' },
        {'role' : ASSISTANT, 'content' : 'number of gestures can be detected' },
    ]

    ## Post-checking prompt
    ### Feedback prompt
    system_message_check_response = """You are an expert in reviewing student answers.
Given text sources, a question, and a student answer, you must step by step evaluate the student answer correction based on text sources.
If the student don't know the answer, just skip the evaluation.
If the answer contains knowledge not mentioned in source, it is wrong.
Examples:
Source:
The following source code will output a standard message along with 'Hello World':
```python
from msg.srv import msg_std
print(msg_std, 'Hello World')
```

Question: 
Write a python code to print out a long message along with 'Hi'.

Student answer:
```python
from msg.srv import msg_long
print(msg_long, 'Hi')
```

Evaluation: The student uses a strange package named `msg_long` which is not included in the source. So the answer is wrong.

"""

    response_check_template = """Source:
{source}

Question:
{question}

Student answer:
{answer}

Evaluation:
"""

    ### Revise answer prompt