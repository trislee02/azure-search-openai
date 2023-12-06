from splitter.splitter import Splitter
from document.Document import Document
import warnings
import openai
import json
import utils
from tenacity import retry, stop_after_attempt, wait_random_exponential, wait_fixed
import re

warnings.filterwarnings("ignore")
from pprint import pprint

PROMPT_SYSTEM_EXTRACTING_QA = """Given emails, create a collection of FAQ by extracting customer questions from Customer message and corresponding answers from LuxAI Support. Please do not modify the LuxAI answer.
Respond in the following valid JSON format.

JSON format of FAQ:
[
{"question": "CUSTOMER QUESTION 1 HERE",
 "answer": "EXTRACTED ANSWER 1 FROM LUXAI"},
 {"question": "CUSTOMER QUESTION 2 HERE",
 "answer": "EXTRACTED ANSWER 2 FROM LUXAI"}
]"""

USER_TEMPLATE_EXTRACTING_QA = """
Given emails:

{emails}

JSON format of FAQ:
"""

def before_retry_sleep(retry_state):
    # print(f"Rate limited on the OpenAI API, sleeping before retrying...")
    try:
        retry_state.outcome.result()
    except Exception as e:
        print(e)

@retry(wait=wait_random_exponential(min=15, max=60), stop=stop_after_attempt(15), before_sleep=before_retry_sleep)
def generate_qa(messages, temperature) -> list[str]:
    list_qa = []
    response = openai.ChatCompletion.create(engine="tri-turbo",
                                            messages = messages,
                                            temperature=temperature,
                                            top_p=0.95,
                                            frequency_penalty=0,
                                            presence_penalty=0,
                                            stop=None)
    print(response.choices[0].message.content)
    qa_json = json.loads(response.choices[0].message.content)
    for qa in qa_json:
        qa_str = f"{qa['question']}\n{qa['answer']}"
        list_qa.append(qa_str)
    return list_qa

def clean_text(text: str) -> str:
    text = text.replace("\n\n", "\n")
    return text

class EmailSplitter(Splitter):
    def __init__(self):
        pass

    def load(self, filename: str) -> list[Document]:
        with open(filename, "r", encoding='windows-1252') as f:
            content = f.read()
            return [Document(content=content)]

    def split(self, documents: list[Document]) -> list[Document]:
        segments = []
        
        all_text = "".join(doc.content for doc in documents)
        all_text = clean_text(all_text)

        user_content = USER_TEMPLATE_EXTRACTING_QA.format(emails=all_text)

        messages = [{"role": "system",  "content": PROMPT_SYSTEM_EXTRACTING_QA},
                    {"role": "user",    "content": user_content}]
        
        qa_list = generate_qa(messages=messages, temperature=0.1)
        for qa in qa_list: segments.append(Document(content=qa))
        return segments

    def create_section(self, filename: str, documents: list[Document]):
        file_id = utils.filename_to_id(filename)

        for i, doc in enumerate(documents):
            section = {
                "id": f"{file_id}-page-{i}",
                "content": doc.content,
                "sourcefile": filename,
                "embedding": utils.compute_embedding(doc.content)
            }
            yield section  
    

    
    def before_retry_sleep(retry_state):
        print(f"Rate limited on the OpenAI API, sleeping before retrying...")

    @retry(wait=wait_random_exponential(min=15, max=60), stop=stop_after_attempt(15), before_sleep=before_retry_sleep)
    def __generate_summary(self, text: str, previous_parts: str) -> str:
        system_message = """Given context of the conversation and current messages/emails, write a concise summary for current message."""
        content_template = """Summary of the preceding parts:
```
{summary_previous_parts}
```
The current emails/messages:
```
{text}
```
Introduction for the current part:"""
        messages = [{"role": "system",  "content": system_message},
                    {"role": "user",    "content": content_template.format(text=text,
                                                                           summary_previous_parts=previous_parts)}]

        response = openai.ChatCompletion.create(engine=self.gptdeployment,
                                                messages = messages,
                                                temperature=0.7,
                                                max_tokens=800,
                                                top_p=0.95,
                                                frequency_penalty=0,
                                                presence_penalty=0,
                                                stop=None)

        return response.choices[0].message.content