
function addTask() {
    const taskInput = document.getElementById("taskInput");
    const taskText = taskInput.value.trim();

    if (taskText === "") {
        alert("Please enter a task.");
        return;
    }

    const li = document.createElement("li");
    const task = document.createElement("span");

    task.className = "task-text";
    task.textContent = taskText;
    task.onclick = function () {
        completeTask(task);
    };

    const editButton = document.createElement("button");
    editButton.className = "edit-btn";
    editButton.textContent = "Edit";
    editButton.onclick = function () {
        editTask(editButton);
    };

    const deleteButton = document.createElement("button");
    deleteButton.className = "delete-btn";
    deleteButton.textContent = "Delete";
    deleteButton.onclick = function () {
        deleteTask(deleteButton);
    };

    li.appendChild(task);
    li.appendChild(editButton);
    li.appendChild(deleteButton);

    document.getElementById("taskList").appendChild(li);

    taskInput.value = "";
    taskInput.focus();
}

function editTask(button) {
    const task = button.parentElement.querySelector(".task-text");
    const newTask = prompt("Edit your task:", task.textContent);

    if (newTask !== null && newTask.trim() !== "") {
        task.textContent = newTask.trim();
    }
}

function deleteTask(button) {
    const task = button.parentElement;
    task.remove();
}

function completeTask(task) {
    task.classList.toggle("completed");
}
