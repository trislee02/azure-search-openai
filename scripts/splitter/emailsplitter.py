from splitter.splitter import Splitter
from document.Document import Document
import warnings
import openai
import json
import utils
from tenacity import retry, stop_after_attempt, wait_random_exponential, wait_fixed
import re
import os

warnings.filterwarnings("ignore")
from pprint import pprint

# EXTRACTING QUESTIONS FROM EMAILS
PROMPT_SYSTEM_EXTRACTING_QA = """Given a list of emails from customers and LuxAI Support team, you are required to create a collection of FAQ by finding questions from Customer emails for every part of LuxAI Support emails.

Requirements:
  - Preserve the context of the customer's question and keep it as much detailed as possible.
  - Preserve the answers from LuxAI Support and extend the answer as many sentences as possible.
  - For parts that cannot find the question, leave the question blank "".
  - The collection of FAQ must cover all LuxAI Support emails.
  
Respond in the following valid JSON format.
JSON format of FAQ:
[
  {"question": "CUSTOMER QUESTION 1 HERE",
   "answer": "ANSWER 1 FROM LUXAI"},
  {"question": "CUSTOMER QUESTION 2 HERE",
   "answer": "ANSWER 2 FROM LUXAI"}
]"""

USER_TEMPLATE_EXTRACTING_QA = """
Given emails:
```
{emails}
```
JSON format of FAQ:
"""

# GENERATE POSSIBLE QUESTIONS
PROMPT_SYSTEM_GENERATE_QUESTION = """Generate one suitable question for each answer in this list of answers.

Respond in the JSON-formatted list as below:
JSON-formatted list of answers:
[
  {"question": "",
   "answer": "GIVEN ANSWER 1"},
  {"question": "",
   "answer": "GIVEN ANSWER 2"}
]

JSON-formatted list:
[
  {"question": "GENERATED QUESTION 1 FOR GIVEN ANSWER 1",
   "answer": "GIVEN ANSWER 1"},
  {"question": "GENERATED QUESTION 2 FOR GIVEN ANSWER 2",
   "answer": "GIVEN ANSWER 2"}
]
"""

USER_TEMPLATE_GENERATE_QUESTION = """
JSON-formatted list of answers:
{answers}

JSON-formatted list:
"""

def before_retry_sleep(retry_state):
    # print(f"Rate limited on the OpenAI API, sleeping before retrying...")
    try:
        retry_state.outcome.result()
    except Exception as e:
        print(e)

@retry(wait=wait_random_exponential(min=15, max=60), stop=stop_after_attempt(15), before_sleep=before_retry_sleep)
def generate_qa(emails: str, temperature: float = 0.0) -> list[str]:
    list_qa = []
    list_answers_only = []

    # Extract question-answer pairs
    print("EXTRACTING QUESTION-ANSWER PAIRS...")
    user_content = USER_TEMPLATE_EXTRACTING_QA.format(emails=emails)

    messages = [{"role": "system",  "content": PROMPT_SYSTEM_EXTRACTING_QA},
                {"role": "user",    "content": user_content}]
    response = openai.ChatCompletion.create(engine="tri-turbo",
                                            messages = messages,
                                            temperature=temperature,
                                            max_tokens=4000,
                                            top_p=0.95,
                                            frequency_penalty=0,
                                            presence_penalty=0,
                                            stop=None)
    print(response.choices[0].message.content)
    qa_json = json.loads(response.choices[0].message.content)
    for qa in qa_json:
        if qa['question'].strip() == "":
            list_answers_only.append(qa)
        else:
            qa_str = f"{qa['question']}\n{qa['answer']}"
            list_qa.append(qa_str)

    # Generate questions for answers
    if len(list_answers_only) > 0:
        print("GENERATING QUESTIONS...")
        pprint(list_answers_only)
        print("========")
        answers = json.dumps(list_answers_only)
        user_content = USER_TEMPLATE_GENERATE_QUESTION.format(answers=answers)

        messages = [{"role": "system",  "content": PROMPT_SYSTEM_GENERATE_QUESTION},
                    {"role": "user",    "content": user_content}]
        response = openai.ChatCompletion.create(engine="tri-turbo",
                                                messages = messages,
                                                temperature=temperature,
                                                max_tokens=4000,
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
    cleaned_text = re.sub(r'\n{2,}', "\n", text)
    cleaned_text = cleaned_text.strip()
    return cleaned_text

class EmailSplitter(Splitter):
    def __init__(self, openaiapikey: str = None, openaikey: str = "", openaiservice: str = "", gptdeployment: str = ""):
        self.openapikey = openaiapikey
        self.openaikey = openaikey
        self.openaiservice = openaiservice
        self.gptdeployment = gptdeployment

    def load(self, filename: str) -> list[Document]:
        with open(filename, "r", encoding='windows-1252') as f:
            content = f.read()
            return [Document(content=content)]

    def split(self, documents: list[Document]) -> list[Document]:
        segments = []
        
        all_text = "".join(doc.content for doc in documents)
        all_text = clean_text(all_text)
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ALL TEXT <<<<<<<<<<<<<<<<<<<<<<<<<<<")
        print(all_text)
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END TEXT <<<<<<<<<<<<<<<<<<<<<<<<<<<")
        
        qa_list = generate_qa(emails=all_text)
        for qa in qa_list: segments.append(Document(content=qa))
        return segments

    def create_section(self, filename: str, documents: list[Document]):
        file_id = utils.filename_to_id(filename)
        file_basename = os.path.basename(filename)
        f_out = open(f"emails/splits-{file_basename}", "w", encoding="utf-8")
        for i, doc in enumerate(documents):
            f_out.write(f"{doc.content}\n-------------\n")
            section = {
                "id": f"{file_id}-page-{i}",
                "content": doc.content,
                "sourcefile": filename,
                "embedding": utils.compute_embedding(doc.content)
            }
            yield section  
        f_out.close()
    